# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath
import time
import scipy.stats
from queue import Queue

# constants
rotatechange = 0.13
speedchange = 0.08
speed_calibration = 0.75
padding = 3
buffer_time = 1
occ_bins = [-1, 0, 51, 100]
stop_distance = 0.5
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
mapthreshold = 50
unknown = 1
unoccupied = 2
occupied = 3

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


# write text to file
def write_to_file(file_name, text):
    f = open(file_name, 'w')
    f.write(repr(text))
    f.close()
    

# check if coordinate is valid given matrix
def valid(coordinate, matrix):
    return coordinate[0] >= 0 and coordinate[0] < len(matrix) and coordinate[1] >= 0 and coordinate[1] < len(matrix[0])


# breath first search
def bfs(matrix, start, end_cond, not_neighbour_cond):
    row = len(matrix)
    col = len(matrix[0])
    visited = np.zeros((row, col), dtype=int)
    parent = np.empty((row, col, 2), dtype=int)
    for i in range(row):
      for j in range(col):
        for k in range(2):
          parent[i][j][k] = -1
    queue = Queue(row * col)
    end = None

    def valid(node):
        return node[0] >= 0 and node[0] < row and node[1] >= 0 and node[1] < col
    
    def getNeighbours(node):
        neighbours = []
        potential_neighbours = []

        top = (node[0]-1, node[1])
        left = (node[0], node[1]-1)
        right = (node[0], node[1]+1)
        bottom = (node[0]+1, node[1])
        top_left = (node[0]-1, node[1]-1)
        top_right = (node[0]-1, node[1]+1)
        bottom_left = (node[0]+1, node[1]-1)
        bottom_right = (node[0]+1, node[1]+1)

        potential_neighbours.append(top)
        potential_neighbours.append(left)
        potential_neighbours.append(right)
        potential_neighbours.append(bottom)
        potential_neighbours.append(top_left)
        potential_neighbours.append(top_right)
        potential_neighbours.append(bottom_left)
        potential_neighbours.append(bottom_right)
        
        for potential_neighbour in potential_neighbours:
            if valid(potential_neighbour) and matrix[potential_neighbour[0]][potential_neighbour[1]] != not_neighbour_cond and visited[potential_neighbour[0]][potential_neighbour[1]] == 0:
                neighbours.append(potential_neighbour)
        return neighbours


    def backtrack():
        path = []
        curr = [end[0], end[1]]
        while curr[0]!= -2 and curr[1] != -2:
            path.append(curr)
            par = [parent[curr[0]][curr[1]][0], parent[curr[0]][curr[1]][1]]
            curr = par
        return path[::-1]
    
    if start[0] < 0 or start[1] < 0 or start[0] >= row or start[1] >= col:
        return []

    visited[start[0]][start[1]] = 1
    parent[start[0]][start[1]] = [-2, -2]
    queue.put(start)

    while not queue.empty():
        curr = queue.get()
      
        if matrix[curr[0]][curr[1]] == end_cond:
            end = curr
            break

        neighbours = getNeighbours(curr)
        for i in range(len(neighbours)):
            visited[neighbours[i][0]][neighbours[i][1]] = 1
            parent[neighbours[i][0]][neighbours[i][1]] = curr
            queue.put(neighbours[i])

    if end != None:
        return backtrack()
    else:
        return []


# calculate angle between start and end coordinate given initial angle
def cal_angle(start, end, initial_angle=0):
    delta_x = end[1] - start[1]
    delta_y = end[0] - start[0]

    if initial_angle < 0:
        modified_initial_angle = 360 + initial_angle
    else:
        modified_initial_angle = initial_angle

    # first quadrant
    if delta_x > 0 and delta_y > 0:
        print('First quadrant')
        return math.degrees(abs(math.atan(delta_y / delta_x))) - modified_initial_angle
    # fourth quadrant
    elif delta_x > 0 and delta_y < 0:
        print('Fourth quadrant')
        return (360 - math.degrees(abs(math.atan(delta_y / delta_x)))) - modified_initial_angle
    # thrid quadrant
    elif delta_x < 0 and delta_y < 0:
        print('Third quadrant')
        return (180 + math.degrees(abs(math.atan(delta_y / delta_x)))) - modified_initial_angle 
    # second quadrant
    elif delta_x < 0 and delta_y > 0:
        print('Second quadrant')
        return (180 - math.degrees(abs(math.atan(delta_y / delta_x)))) - modified_initial_angle 
    # up
    elif delta_x == 0 and delta_y > 0:
        print('Up')
        return 90 - modified_initial_angle
    # down
    elif delta_x == 0 and delta_y < 0:
        print('Down')
        return 270 - modified_initial_angle
    elif delta_x > 0 and delta_y == 0:
        print('Right')
        return 0 - modified_initial_angle
    # left
    elif delta_x < 0 and delta_y == 0:
        print('Left')
        return 180 - modified_initial_angle 
    # do not change
    else:
        return 0


# generate shorter path given path by taking only points of diffent angles
def shorter_path(path):
    # optimization
    if len(path) <= 2:
        return path

    # add starting point to returned path
    shorter_path = [path[0]]

    # add intermediate points of different angles to returned path
    angle = cal_angle(path[0], path[1])
    for i in range(1, len(path) - 1):
        if angle != cal_angle(path[i], path[i + 1]):
            shorter_path.append(path[i])
            angle = cal_angle(path[i], path[i + 1])

    # add ending point to returned path
    shorter_path.append(path[-1])

    return shorter_path


def getNeighbours(node, matrix):
    neighbours = []
    potential_neighbours = []

    top = (node[0]-1, node[1])
    left = (node[0], node[1]-1)
    right = (node[0], node[1]+1)
    bottom = (node[0]+1, node[1])
    top_left = (node[0]-1, node[1]-1)
    top_right = (node[0]-1, node[1]+1)
    bottom_left = (node[0]+1, node[1]-1)
    bottom_right = (node[0]+1, node[1]+1)

    potential_neighbours.append(top)
    potential_neighbours.append(left)
    potential_neighbours.append(right)
    potential_neighbours.append(bottom)
    potential_neighbours.append(top_left)
    potential_neighbours.append(top_right)
    potential_neighbours.append(bottom_left)
    potential_neighbours.append(bottom_right)
    
    for potential_neighbour in potential_neighbours:
        if valid(potential_neighbour, matrix):
            neighbours.append(potential_neighbour)
    return neighbours


def shorter_path_v2(path, matrix):
    if len(path) <= 2:
        return path

    shorter_path = [path[0]]
    for i in range(1, len(path) - 1):
        point = path[i]
        neighbours = getNeighbours(point, matrix)
        number_of_neighbours = 0
        for j in range(len(neighbours)):
            neighbour = neighbours[j]
            number_of_neighbours += matrix[neighbour[0]][neighbour[1]]
        if number_of_neighbours == 2:
            shorter_path.append(neighbour)

    shorter_path.append(path[-1])

    return shorter_path


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create class variables
        self.cur_pos = [-1, -1, -1]
        self.grid_x = -1
        self.grid_y = -1
        self.map_res = 0
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.occdata = np.array([])
        self.encoded_msgdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # log to console if occ_callback() is called
        # self.get_logger().info('In occ_callback')

        write_to_file('raw_msgdata.txt', msg.data)

        # create numpy array from original map data
        msgdata = np.array(msg.data)
        write_to_file('numpy_msgdata.txt', msgdata)

        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # encode map data by converting -1 to 1, [0:50] to 2, [51:100] to 3
        occ_counts, edges, msgdata = scipy.stats.binned_statistic(msgdata, np.nan, statistic='count', bins=occ_bins)
        write_to_file('encoded_msgdata.txt', msgdata)

        # reshape 1d map data to 2d
        self.occdata = np.int8(msgdata.reshape(msg.info.height,msg.info.width))
        np.savetxt('2d_msgdata.txt', self.occdata)

        # pad map data
        self.encoded_msgdata = np.int8(msgdata.reshape(msg.info.height,msg.info.width))
        for i in range(len(self.encoded_msgdata)):
            for j in range(len(self.encoded_msgdata[0])):
                if self.occdata[i][j] == occupied:
                    node = [i, j]
                    for k in range(-padding, padding + 1):
                        for l in range(-padding, padding + 1):
                            test_node = [i+k, j+l]
                            if test_node != node and valid(test_node, self.encoded_msgdata):
                                self.encoded_msgdata[test_node[0]][test_node[1]] = occupied
        np.savetxt('padded_msgdata.txt', self.encoded_msgdata)

        # get transformation
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
       
        # get map information
        self.map_res = msg.info.resolution
        map_origin = msg.info.origin.position

        # get turtlebot current position
        self.cur_pos = trans.transform.translation
        self.grid_y = round((self.cur_pos.x - map_origin.x) / self.map_res)
        self.grid_x = round(((self.cur_pos.y - map_origin.y) / self.map_res))
        # encode turtlebot current position on map data
        self.occdata[self.grid_x][self.grid_y] = 4

        # get turtlebot current orientation
        cur_rot = trans.transform.rotation
        cur_row, cur_pitch, cur_yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current yaw: %f degree' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired yaw: %f degree' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        #self.get_logger().info('c_change_dir: %i' % c_change_dir)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        #self.get_logger().info('c_dir_diff: %i' % c_dir_diff)
        self.get_logger().info('End yaw: %f degree' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        self.get_logger().info('In pick_direction')

        # if occ_callback() has not been called before, pick mapped direction with the largest lidar value
        if self.grid_x == -1 or self.grid_y == -1:
            if self.laser_range.size != 0:
                # use nanargmax as there are nan's in laser_range added to replace 0's
                lr2i = np.nanargmax(self.laser_range)
                self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
            else:
                lr2i = 0
                self.get_logger().info('No data!')

            # rotate to that direction
            self.rotatebot(float(lr2i))

            # start moving
            self.get_logger().info('Start moving')
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)

        # if occ_callback() has been called before, pick nearest unmapped direction
        if self.grid_x != -1 and self.grid_y != -1:
            # generate path using bfs
            path = bfs(self.encoded_msgdata, [self.grid_x, self.grid_y], 1, 3)
            np.savetxt('path.txt', path)

            # if path generated is empty, navigation is completed!
            if len(path) == 0:
                self.get_logger().info('Fully mapped!')
                self.stopbot()

            # generate shorter path
            short_path = shorter_path(path)
            #short_path = shorter_path_v2(path, self.encoded_msgdata)
            np.savetxt('shorter_path.txt', short_path)

            # encode shorter path on map data
            for points in short_path:
                self.encoded_msgdata[points[0]][points[1]] = 5
            # encode ending point
            self.encoded_msgdata[short_path[-1][0]][short_path[-1][1]] = 7
            np.savetxt('map_with_shorter_path.txt', self.encoded_msgdata)
            # auto plot map with shorter path
            plt.imshow(self.encoded_msgdata, cmap='gray')
            #plt.draw()
            plt.pause(0.00000000001)

            # encode path on map data
            for points in path:
                self.encoded_msgdata[points[0]][points[1]] = 5
            # encode ending point
            self.encoded_msgdata[short_path[-1][0]][short_path[-1][1]] = 7
            np.savetxt('map_with_path.txt', self.encoded_msgdata)

            # turtlebot move according to the shorter path generated
            curr_point = short_path[0]
            for point in short_path[1:]:
                # rotate turtlebot
                angle = cal_angle(curr_point, point, math.degrees(self.yaw))
                if angle < 0:
                    self.get_logger().info('Rotation: %f degree clockwise' % angle)
                elif angle > 0:
                    self.get_logger().info('Rotation: %f degree anticlockwise' % angle)
                self.rotatebot(angle) 

                # move forward
                self.get_logger().info('Start moving')
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                time.sleep(1)
                self.publisher_.publish(twist)

                # check if desired coordinate has reached
                #distance = ((curr_point[0] - point[0]) ** 2 + (curr_point[1] - point[1]) ** 2) ** 0.5 
                #self.get_logger().info('Current coordinate: %i, %i' % (curr_point[0], curr_point[1]))
                #self.get_logger().info('Desired coordinate: %i, %i' % (point[0], point[1]))
                #self.get_logger().info('Distance: %i' % distance)
                #while (((self.grid_x - curr_point[0]) ** 2 + (self.grid_y - curr_point[1]) ** 2) ** 0.5) < distance:
                #    rclpy.spin_once(self)
                    
                    # in case the turtlebot does not follow the path generated and going to crash into the wall
                    #if self.laser_range.size != 0:
                    #    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    #    if(len(lri[0])>0):
                    #        self.stopbot()
                    #        self.get_logger().info('Too far!')
                    #        self.pick_direction()

                #    continue

                # calculate distance
                rclpy.spin_once(self)
                distance = ((curr_point[0] - point[0]) ** 2 + (curr_point[1] - point[1]) ** 2) ** 0.5 
                calibrated_distance = distance * self.map_res
                #calibrated_distance = distance
                self.get_logger().info('self.map_res: %f' % self.map_res)
                self.get_logger().info('Distance: %f m' % calibrated_distance)
                # calculate speed
                calibrated_speed = speedchange * speed_calibration
                self.get_logger().info('Speed: %f m/s' % calibrated_speed)
                # calculate time
                calibrated_time = calibrated_distance / calibrated_speed + buffer_time
                self.get_logger().info('Time: %f s' % calibrated_time)
                time.sleep(calibrated_time)
                self.get_logger().info('Done moving forward')

                # update current point
                #self.get_logger().info('End coordinate: %i, %i' % (self.grid_x, self.grid_y))
                #curr_point = [self.grid_x, self.grid_y]
                curr_point = [point[0], point[1]]


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

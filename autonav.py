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
import math
import cmath
import time
import scipy.stats
from queue import Queue

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 51, 100]
stop_distance = 0.25
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


# a utility method to save value to filename
def save_to_file(filename, value):
    f = open(filename, 'w')
    f.write(repr(value))
    f.close()
    

def valid(node, matrix):
    return node[0] >= 0 and node[0] < len(matrix) and node[1] >= 0 and node[1] < len(matrix[0])


def bfs(matrix, start, end_cond, not_neighbour_cond):
    row = len(matrix)
    col = len(matrix[0])
    visited = np.zeros((row, col), dtype=int)
    #for i in range(row):
    #  for j in range(col):
    #    visited[i][j] = 0
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
        #if valid(left) and matrix[left[0]][left[1]] != not_neighbour_cond and visited[left[0]][left[1]] == 0:
        #    neighbours.append(left)
        #if valid(right) and matrix[right[0]][right[1]] != not_neighbour_cond and visited[right[0]][right[1]] == 0:
        #    neighbours.append(right)
        #if valid(top) and matrix[top[0]][top[1]] != not_neighbour_cond and visited[top[0]][top[1]] == 0:
        #    neighbours.append(top)
        #if valid(bottom) and matrix[bottom[0]][bottom[1]] != not_neighbour_cond and visited[bottom[0]][bottom[1]] == 0:
        #    neighbours.append(bottom)
        #for i in range(-1, 2):
        #    for j in range(-1, 2):
        #        test_node = [node[0]+i, node[1]+j]
        #        if test_node != node and valid(test_node) and matrix[test_node[0]][test_node[1]] != not_neighbour_cond and visited[test_node[0]][test_node[1]] == 0:
        #            neighbours.append(test_node)
        return neighbours


    def backtrack():
        path = []
        curr = [end[0], end[1]]
        #while curr[0]!= -2.0 and curr[1] != -2.0:
        #    path.append(curr)
        #    par = [parent[int(curr[0])][int(curr[1])][0], parent[int(curr[0])][int(curr[1])][1]]
        #    curr = par
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


#def cal_angle(start, end, default_angle=0):
#    delta_x = end[1] - start[1]
#    delta_y = end[0] - start[0]
#    if delta_x > 0 and delta_y > 0:
#        return math.degrees(math.atan(delta_y / delta_x)) - default_angle
#    elif delta_x > 0 and delta_y < 0:
#        return math.degrees(math.atan(delta_y / delta_x)) + 90 - default_angle
#    elif delta_x < 0 and delta_y < 0:
#        return math.degrees(math.atan(delta_y / delta_x)) + 180 - default_angle
#    elif delta_x < 0 and delta_y > 0:
#        return math.degrees(math.atan(delta_y / delta_x)) + 270 - default_angle
#    elif delta_x == 0 and delta_y > 0:
#        return 90 - default_angle
#    elif delta_x == 0 and delta_y < 0:
#        return 270 - default_angle
#    elif delta_x > 0 and delta_y == 0:
#        return 0 - default_angle
#    elif delta_x > 0 and delta_y == 0:
#        return 180 - default_angle
#    else:
#        return 0 - default_angle


#def cal_angle(start, end, def_angle=0):
#    delta_x = end[0] - start[0]
#    delta_y = end[1] - start[1]

#    if def_angle < 0:
#        default_angle = 360 + def_angle
#    else:
#        default_angle = def_angle

#    if delta_x > 0 and delta_y > 0:
#        return math.degrees(math.atan(delta_y / delta_x)) - 90 - default_angle + 90
#    elif delta_x > 0 and delta_y < 0:
#        return math.degrees(math.atan(delta_y / delta_x)) + 90 - 90 - default_angle + 90
#    elif delta_x < 0 and delta_y < 0:
        #return math.degrees(math.atan(delta_y / delta_x)) + 180 - 90 - default_angle + 90
#        return -(math.degrees(math.atan(delta_y / delta_x)) + 180) + default_angle 
#    elif delta_x < 0 and delta_y > 0:
#        return math.degrees(math.atan(delta_y / delta_x)) + 270 - 90 - default_angle + 90
#    elif delta_x == 0 and delta_y > 0:
        #return 90 - 90 - default_angle + 90
#        return 90 - default_angle + 90
#    elif delta_x == 0 and delta_y < 0:
        #return 270 - 90 - default_angle + 90
#        return 180 - 90 - default_angle + 90
#    elif delta_x > 0 and delta_y == 0:
        #return 0 - 90 - default_angle + 90
#        return -90 - 90 - default_angle + 90
#    elif delta_x < 0 and delta_y == 0:
        #return 180 - 90 - default_angle + 90
#        return 90 - 90 - default_angle + 90
#    else:
        #return 0 - 90 - default_angle + 90
#        return 0
        

def cal_angle(start, end, def_angle=0):
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]

    if def_angle < 0:
        default_angle = 360 + def_angle
    else:
        default_angle = def_angle
    #default_angle = def_angle

    if delta_x > 0 and delta_y > 0:
        #return -math.degrees(math.atan(delta_y / delta_x)) - default_angle 
        return default_angle - math.degrees(math.atan(delta_y / delta_x)) 
    elif delta_x > 0 and delta_y < 0:
        #return math.degrees(math.atan(delta_y / delta_x)) - 180 - default_angle
        return default_angle - (180 - math.degrees(math.atan(delta_y / delta_x)))
    elif delta_x < 0 and delta_y < 0:
        #return math.degrees(math.atan(delta_y / delta_x)) - 90 - default_angle 
        return default_angle - (180 + math.degrees(math.atan(delta_y / delta_x)))
    elif delta_x < 0 and delta_y > 0:
        #return math.degrees(math.atan(delta_y / delta_x)) + 90 - default_angle
        return default_angle - (360 - math.degrees(math.atan(delta_y / delta_x)))
    elif delta_x == 0 and delta_y > 0:
        #return 0 - default_angle
        return -90 - default_angle
    elif delta_x == 0 and delta_y < 0:
        return 90 - default_angle
    elif delta_x > 0 and delta_y == 0:
        #return 90 - default_angle
        return -default_angle
    elif delta_x < 0 and delta_y == 0:
        #return -90 - default_angle 
        return -180 - default_angle 
    else:
        return 0


def shorter_path(path):
    if len(path) <= 2:
        return path
    shorter_path = [path[0]]
    angle = cal_angle(path[0], path[1])
    for i in range(1, len(path) - 1):
        if angle != cal_angle(path[i], path[i + 1]):
            shorter_path.append(path[i])
            angle = cal_angle(path[i], path[i + 1])
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
        #self.get_logger().info('self_yaw: %i' % self.yaw)


    def occ_callback(self, msg):
        # log to console if occ_callback() is called
        #self.get_logger().info('In occ_callback')
        # save orginal msg.data to file
        save_to_file('raw_msgdata.txt', msg.data)
        # create numpy array from original msg.data
        msgdata = np.array(msg.data)
        # save numpy array msgdata to file
        save_to_file('numpy_msgdata.txt', msgdata)

        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # convert -1 to 1, [0:50] to 2, [51:100] to 3
        #for i in range(len(msgdata)):
        #    if msgdata[i] == -1:
        #        msgdata[i] = unknown
        #    elif msgdata[i] <= mapthreshold:
        #        msgdata[i] = unoccupided
        #    elif msgdata[i] > mapthreshold:
        #        msgdata[i] = occupided
        occ_counts, edges, msgdata = scipy.stats.binned_statistic(msgdata, np.nan, statistic='count', bins=occ_bins)
        # save modified numpy array msgdata to file
        save_to_file('encoded_msgdata.txt', msgdata)
        # reshape 1d msgdata to 2d
        self.occdata = np.int8(msgdata.reshape(msg.info.height,msg.info.width))
        self.encoded_msgdata = np.int8(msgdata.reshape(msg.info.height,msg.info.width))
        # check whether map fully mapped
        #self.get_logger().info('fully mapped: %s' % str(self.is_fully_mapped()))
        # save 2d msgdata to file
        np.savetxt('2d_msgdata.txt', self.occdata)
        for i in range(len(self.encoded_msgdata)):
            for j in range(len(self.encoded_msgdata[0])):
                if self.occdata[i][j] == occupied:
                    node = [i, j]
                    for k in range(-4, 5):
                        for l in range(-4, 5):
                            test_node = [i+k, j+l]
                            if test_node != node and valid(test_node, self.encoded_msgdata):
                                self.encoded_msgdata[test_node[0]][test_node[1]] = occupied
        np.savetxt('padded_msgdata.txt', self.encoded_msgdata)

        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
       
        self.cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation

        self.map_res = msg.info.resolution
        map_origin = msg.info.origin.position

        cur_row, cur_pitch, cur_yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        #self.get_logger().info('turtlebot current direction: %i degree' % math.degrees(cur_yaw))

        self.grid_y = round((self.cur_pos.x - map_origin.x) / self.map_res)
        self.grid_x = round(((self.cur_pos.y - map_origin.y) / self.map_res))
        #self.get_logger().info('turtlebot current position: (%i, %i)' % (self.grid_x, self.grid_y))

        self.occdata[self.grid_x][self.grid_y] = 4

        np.savetxt(mapfile, self.occdata)

        #path = bfs(self.occdata, [grid_x, grid_y], 1, 3)
        #for points in path:
        #    self.occdata[int(points[0])][int(points[1])] = 5
        #np.savetxt('map_with_path.txt', self.occdata)
        #self.get_logger().info('path length: %i' % len(path))
        #self.get_logger().info('map size: %i x %i' % (len(self.occdata), len(self.occdata[0])))
        #self.get_logger().info('ending point: (%i, %i)' % (path[len(path)-1][0], path[len(path)-1][1]))
        #np.savetxt('path.txt', path)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        

    #def is_fully_mapped(self):
    #    #self.get_logger().info('in is_fully_mapped1')
    #    row = len(self.occdata)
    #    col = len(self.occdata[0])
    #    #self.get_logger().info('in is_fully_mapped2')
    #    # iterate over the whole occdata
    #    for i in range(row):
    #        #self.get_logger().info('in is_fully_mapped3')
    #        for j in range(col):
    #            #self.get_logger().info('in is_fully_mapped4')
    #            if self.occdata[i][j] == 0:
    #                # get neighbouring values
    #                #self.get_logger().info('in is_fully_mapped5')
    #                topleft = 0 if (i == 0 or j == 0) else self.occdata[i-1][j-1]
    #                top = 0 if i == 0 else self.occdata[i-1][j]
    #                topright = 0 if (i == 0 or j == col) else self.occdata[i-1][j+1]
    #                left = 0 if j == 0 else self.occdata[i][j-1]
    #                right = 0 if j == col else self.occdata[i][j+1]
    #                bottomleft = 0 if (i == row or j == 0) else self.occdata[i+1][j-1]
    #                bottom = 0 if i == row else self.occdata[i+1][j]
    #                bottomright = 0 if (i == row or j == col) else self.occdata[i+1][j+1]
    #                #self.get_logger().info('in is_fully_mapped6')
    #                # check whether any one of the neighbouring values is -1
    #                return topleft == -1 or top == -1 or topright == -1 or left == -1 or right == -1 or bottomleft == -1 or bottom == -1 or bottomright == -1


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f degree' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f degree' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        self.get_logger().info('c_change_dir: %i' % c_change_dir)
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

        self.get_logger().info('c_dir_diff: %i' % c_dir_diff)
        self.get_logger().info('End Yaw: %f degree' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        self.get_logger().info('In pick_direction')
        if self.grid_x == -1 and self.grid_y == -1:
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

        if self.grid_x != -1 and self.grid_y != -1:
            path = bfs(self.encoded_msgdata, [self.grid_x, self.grid_y], 1, 3)
            short_path = shorter_path(path)

            for points in short_path:
                #self.encoded_msgdata[int(points[0])][int(points[1])] = 5
                self.encoded_msgdata[points[0]][points[1]] = 5
            np.savetxt('map_with_shorter_path.txt', self.encoded_msgdata)
            #self.get_logger().info('shorter path length: %i' % len(short_path))
            np.savetxt('shorter_path.txt', short_path)

            for points in path:
                #self.encoded_msgdata[int(points[0])][int(points[1])] = 5
                self.encoded_msgdata[points[0]][points[1]] = 5
            np.savetxt('map_with_path.txt', self.encoded_msgdata)
            #self.get_logger().info('path length: %i' % len(path))
            np.savetxt('path.txt', path)

            #self.get_logger().info('ending point: (%i, %i)' % (path[len(path)-1][0], path[len(path)-1][1]))
            #self.get_logger().info('map size: %i x %i' % (len(self.encoded_msgdata), len(self.encoded_msgdata[0])))
        
            curr_point = short_path[0]
            for point in short_path[1:]:
                #angle = cal_angle(curr_point, point, math.degrees(self.yaw))
                angle = cal_angle([curr_point[1], curr_point[0]], [point[1], point[0]], math.degrees(self.yaw))
                #angle = cal_angle(curr_point, point)
                self.get_logger().info('start point: %i, %i' % (curr_point[1], curr_point[0]))
                self.get_logger().info('end point: %i, %i' % (point[1], point[0]))
                self.get_logger().info('current direction: %i' % self.yaw)
                self.get_logger().info('rotation required: %i' % angle)
                self.rotatebot(angle) 
                self.get_logger().info('Start moving')
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                time.sleep(1)
                self.publisher_.publish(twist)
                #time.sleep(self.map_res / speedchange)

                good_enuf_pos = []
                for i in range(-2, 3):
                    for j in range(-2, 3):
                        test_point = [point[0]+i, point[1]+j]
                        #test_point = [point[1]+i, point[0]+j]
                        if test_point != point and valid(test_point, self.encoded_msgdata):
                            good_enuf_pos.append(test_point)
                np.savetxt('good_enuf_pos.txt', good_enuf_pos)

                #while self.cur_pos != point:
                while [self.grid_x, self.grid_y] not in good_enuf_pos:
                    rclpy.spin_once(self)
                    self.get_logger().info('current position: %i, %i' % (self.grid_x, self.grid_y))
                    self.get_logger().info('desired position: %i, %i' % (point[0], point[1]))
                    self.get_logger().info('rotating')
                    continue

                self.get_logger().info('out of loop coordinate: %i, %i' % (self.grid_x, self.grid_y))
                curr_point = [self.grid_x, self.grid_y]


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
    #auto_nav.rotatebot(1)
    #auto_nav.rotatebot(180)
    #auto_nav.rotatebot(180)
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

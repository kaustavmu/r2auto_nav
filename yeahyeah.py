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
import numpy as np
import math
import cmath
import time
import scipy.stats
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from queue import Queue


# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map2.txt'

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

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
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
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
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
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))
        occ_counts, edges, msgdata = scipy.stats.binned_statistic(msgdata, np.nan, statistic='count', bins=occ_bins)
        # make msgdata go from 0 instead of -1, reshape into 2D
        #oc2 = binnum + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(msgdata.reshape(msg.info.height,msg.info.width))

        #Transforms the data to get the position of Turtlebot
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
             #*********LOOK LATER*********
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        cur_pos = trans.transform.translation
 
        cur_rot = trans.transform.rotation

        map_res = msg.info.resolution

        map_origin = msg.info.origin.position

        grid_x = round((cur_pos.x - map_origin.x) / map_res)

        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        self.occdata[grid_y][grid_x] = 0
	
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
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
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
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

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
       
        #The grand bfs coded by Xin Yi and optimized by yours Truly. This uses a queue to chec possible travel nodes given their equal weight in the map.
        def bfs(matrix, start, end_cond, not_neighbour_cond):
            row = len(matrix)
            col = len(matrix[0])
            visited = np.empty((row, col))
            for i in range(row):
                for j in range(col):
                    visited[i][j] = 0
            parent = np.empty((row, col, 2))
            for i in range(row):
                for j in range(col):
                    for k in range(2):
                        parent[i][j][k] = -1
            queue = Queue(row * col)
            end = None
            
            #Function to check validity of node, to be considered for pathwaymaking
            def valid(node):
                return node[0] >= 0 and node[0] < row and node[1] >= 0 and node[1] < col


            #Function to get neighbours. Made a small adjustment to the end_cond line where is used np.round to round the result to end_condition
            def getNeighbours(node):
                neighbours = []
                #top_left = (node[0]-1, node[1]-1)
                #top = (node[0]-1, node[1])
                #top_right = (node[0]-1, node[1]+1)
                #left = (node[0], node[1]-1)
                #right = (node[0], node[1]+1)
                #botton_left = (node[0]+1, node[1]-1)
                #bottom = (node[0]+1, node[1])
                #bottom_right = (node[0]+1, node[1]+1)
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        test_node = [node[0]+i, node[1]+j]
                        if test_node != node and valid(test_node) and np.round(matrix[test_node[0]][test_node[1]]) != not_neighbour_cond and visited[test_node[0]][test_node[1]] == 0:
                            neighbours.append(test_node)
                return neighbours


            #Hardcoded possibilities for the neighbours being in certain conformations. Trust me on this bit for now :)
            def diamonds(z):
                chainz = getNeighbours(z)
                rollie = []
                for c in chainz:
                    rollie.append(int(round(self.occdata[c[0]][c[1]])))        
                if rollie[0] == 3:
                    if rollie[1] == rollie[2] == rollie[3] == rollie[4] == rollie[5] == rollie[6] == rollie[7]:
                        return True
                    elif rollie[0] == rollie[1] == rollie[3]:
                        return True
                if rollie[2] == 3:
                    if rollie[0] == rollie[1] == rollie[3] == rollie[4] == rollie[5] == rollie[6] == rollie[7]:
                        return True
                    if rollie[2] == rollie[1] == rollie[4]:
                        return True
                if rollie[5] == 3:
                    if rollie[0] == rollie[1] == rollie[2] == rollie[3] == rollie[4] == rollie[6] == rollie[7]:
                        return True
                    elif rollie[5] == rollie[3] == rollie[6]:
                        return True
                if rollie[7] == 3:
                    if rollie[0] == rollie[1] == rollie[2] == rollie[3] == rollie[4] == rollie[5] == rollie[6]:
                        return True
                    elif rollie[7] == rollie[4] == rollie[6]:
                        return True
                if rollie[1] == 3:
                    if rollie[0] == rollie[2] == rollie[3] == rollie[4] == rollie[5] == rollie[6] == rollie[7]:
                        return True 
                if rollie[3] == 3:
                    if rollie[0] == rollie[1] == rollie[2] == rollie[4] == rollie[5] == rollie[6] == rollie[7]:
                        return True
                if rollie[4] == 3:
                    if rollie[0] == rollie[1] == rollie[2] == rollie[3] == rollie[5] == rollie[6] == rollie[7]:
                        return True
                if rollie[6] == 3:
                    if rollie[0] == rollie[1] == rollie[2] == rollie[3] == rollie[4] == rollie[5] == rollie[7]:
                        return True   
                return False


            #This is the path shortener which shortens output to just vertices for ease of computation!
            def vvs(path):
                newpath = []
                newpath.append(path[0])
                for elem in path[1:len(path)]:
                    if diamonds(elem) == True:
                        newpath.append(elem)
                        continue
                    else:
                        continue                
                newpath.append(path[-1])
                return newpath

            #This function backtracks to find parent nodes until you reach the start, before reversing the path to get a path from start to end.
            def backtrack():
                path = []
                curr = [end[0], end[1]]
                while curr[0]!= -2.0 and curr[1] != -2.0:
                    path.append(curr)
                    par = [parent[int(curr[0])][int(curr[1])][0], parent[int(curr[0])][int(curr[1])][1]]
                    curr = par
                path.reverse() 
                return vvs(path)

            #Returns empty path upon invalid start point
            if start[0] < 0 or start[1] < 0 or start[0] >= row or start[1] >= col:
                return []

            #Prepares queue for iteration
            visited[start[0]][start[1]] = 1
            parent[start[0]][start[1]] = [-2, -2]
            queue.put(start)

            #Okay, it realy starts here. Queue iterations begins 
            while not queue.empty():
                curr = queue.get()
      
                #Modified line with np.round so I can use a simpler end_cond
                if np.round(matrix[curr[0]][curr[1]]) == end_cond:
                    end = curr
                    break

                #Updates visited neighbours and continues the serach for the next node in the queue
                neighbours = getNeighbours(curr)
                for i in range(len(neighbours)):
                    visited[neighbours[i][0]][neighbours[i][1]] = 1
                    parent[neighbours[i][0]][neighbours[i][1]] = curr
                    queue.put(neighbours[i])

            #Returns a valid path if the end condition is satified by a known node.
            if end != None:
                return backtrack()
            #Returns no path otherwise
            else:
                newpath = []
                return newpath       
         
        #This bit is to get the current position of turtlebot in array(this is a numpy.ndarray)
        #Double-check bfs arguments to see if they are correct
        x = np.where(np.round(self.occdata) == 0.0)
        racks = np.asarray(x).T.tolist
        greens = bfs(self.occdata,[racks[0][0],racks[0][1]],-1.0,1.0 or -1.0)
    


        # self.get_logger().info('In pick_direction')
        #if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            #lr2i = np.nanargmax(self.laser_range)
            #self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        #else:
            #lr2i = 0
            #self.get_logger().info('No data!')

        #Functions below talk about t, the rotating angle. t values assume that rotation is clockwise and in radians.
        for h in range(len(greens)-1):
            # rotate to that direction
            q,w,e,r = racks[0][0],racks[0][1],newpath[h+1][0],newpath[h+1][1]
            if r - w == 0:
                if (e-q) > 0:
                    t = 3*(math.pi)/4
                elif (e-q) < 0:
                    t = (math.pi)/4
            else:
                t = math.atan((e-q)/(r-w))
                if (e-q) < 0:
                    if t > 0:
                        t = math.pi - t
                    elif t < 0:
                        t = -t
                elif (e-q) > 0:
                    if t > 0:
                        t = math.pi + t
                elif (e-q) == 0 and (r-w) < 0:
                    t = math.pi
            self.rotatebot(float(t))

            # start moving
            self.get_logger().info('Start moving')
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)

            #Conditional iteration to check for robot being in position at vertex
            while racks[0][0] != e and racks[0][1] != r:
                continue
            else:
                self.stopbot()
            continue
            #wanted to put a function to break the loop is robot is too far, but not sure how.
	


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

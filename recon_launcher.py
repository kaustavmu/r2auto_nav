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
from std_msgs.msg import String

import time
import pigpio
import busio
import board
import adafruit_amg88xx

pi = pigpio.pi()


#setup for launcher & rack
en = 21
in2 = 5
in1 = 6
d2 = 19
d1 = 13
sf = 16

#pitch control 
forwardp = 2000
backp = 1000
stopp = 1500
pitchpin = 18
pi.set_servo_pulsewidth(pitchpin, stopp)

#rack control
forward = 2000 #2000
backward = 1000 #1000
stop = 1500 #1500
servo_pin = 4
pi.set_servo_pulsewidth(servo_pin, stop)

#motor control
pi.write(in2, 0)
pi.write(in1, 1)
pi.write(d2, 1)
i.write(in1, 1)
pi.write(d2, 1)
pi.write(d1, 0)


THRESHOLDTEMP = 32

# setup i2c for ir
ir_power_pin = 17
pi.write(ir_power_pin, 1)
time.sleep(1)
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # create publisher for data about target
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create subscriber to tell launcher when it can fire
        self.launcher_ready_subscription = self.create_subscription(
            String,
            'launcher_ready',
            self.launcher_ready_callback,
            10)
        self.launcher_ready_subscription # prevent unused variable warning
        self.ready = False
        self.target_hit = False

        # once the bot is ready and has informed us, set self.ready to true so we can fire
    def launcher_ready_callback(self, msg):
        self.get_logger().info('Received fr launcher_ready: "%s"' % msg.data)
        if (msg.data == 'launcher ready'):
            self.ready = True
        else:
            self.ready = False
        # periodically publish information about the target 
    def timer_callback(self):
        if (not self.target_hit):
            data = String()
            data = self.detect_target()
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            if (data == 'on target' and self.ready):
                self.fire_ping_pong()
        return
    
    # identify if there is a target and where it is
    def detect_target(self):
        i = 0
        max_temp = 0
        combined_row = [0 for i in range(8)]
        for row in amg.pixels:
            for temp in row:
        #        print(['{0:.1f}'.format(temp)], end = " ")
                if (temp > max_temp):
                    max_temp = temp
                combined_row[i] += temp
         #   print()
            i += 1
        #print(combined_row)
        # find out if target is left, right or center
        weight_left = (combined_row[0] + combined_row[1] + combined_row[2] + combined_row[3]) /4 * 3
        weight_center = combined_row[4] * 3
        weight_right = (combined_row[5] + combined_row[6] + combined_row[7])
        # print(max_temp)
        if (self.target_hit):
            return 'target hit'
        elif (max_temp < THRESHOLDTEMP):
            return 'not found'
        elif (weight_center > weight_left and weight_center > weight_right):
            return 'on target'
        elif (weight_left > weight_center and weight_left > weight_right):
            return 'right'
        elif (weight_right > weight_center):
            return 'left'
        else:
            return '?'

        # once we are on target and the bot is ready, pitch to the appropriate angle and fire
    def fire_ping_pong(self):
        time.sleep(2)
        if (not self.target_hit):
            # aim pitching
            combined_col = [0 for i in range(8)]
            middle = False;
            count = 0
            count_up = 0
            count_down = 0
            # pitch up/ down until the target is in the middle
            while(not middle and count < 5):
                max_temp = 0
                for row in amg.pixels:
                    col_i = 0;
                    for temp in row:
                        #print(['{0:.1f}'.format(temp)], end = " ")
                        combined_col[col_i] += temp
                        if (temp > max_temp):
                            max_temp = temp
                        col_i += 1;
                    #print()
                weight_up = (combined_col[0] + combined_col[1] + combined_col[2] + combined_col[3] + combined_col[4]) * 3/5
                weight_middle = (combined_col[5]) * 3
                weight_down = (combined_col[6] + combined_col[7]) * 3/2
                # print(max_temp)
                if (max_temp > THRESHOLDTEMP):
                    if (weight_middle > weight_up and weight_middle > weight_down):
                        print("target middle")
                        middle = True
                    elif (weight_up > weight_middle and weight_up > weight_down):
                        print("target up")
                        pi.set_servo_pulsewidth(pitchpin, forwardp)
                        time.sleep(0.03)
                        pi.set_servo_pulsewidth(pitchpin, stopp)
                        time.sleep(0.2)
                        count_up += 1
                    elif(weight_down > weight_middle):
                        print("target down")
                        pi.set_servo_pulsewidth(pitchpin, backp)
                        time.sleep(0.04)
                        pi.set_servo_pulsewidth(pitchpin, stopp)
                        time.sleep(0.2)
                        count_down += 1
                    else:
                        print('not sure')
                count += 1
                time.sleep(0.2)

            # firing sequence
            pi.write(en,1)                   # start flywheels
            for i in range (3):
                time.sleep(0.5)
                pi.set_servo_pulsewidth(servo_pin, backward)     # move rack back
                time.sleep(0.21)
                pi.set_servo_pulsewidth(servo_pin, stop)         # stop rack movement
                time.sleep(0.5)
                pi.set_servo_pulsewidth(servo_pin, forward)      # move rack forward to push ball
                time.sleep(0.23)
                pi.set_servo_pulsewidth(servo_pin, stop)         # stop rack movement
                time.sleep(0.5)
            pi.write(en, 0)                  # stop flywheels
        self.target_hit = True

        # reset the launcher to be vertically upright
        count_up_net = count_up - count_down
        #  print(count_up_net)
        # if net move up, shift down
        if (count_up_net > 0):
            while (count_up_net != 0):
                pi.set_servo_pulsewidth(pitchpin, backp)
                time.sleep(0.04)
                pi.set_servo_pulsewidth(pitchpin, stopp)
                time.sleep(0.4)
                count_up_net -= 1
                # print(count_up_net)
        # if net move down, shift up
        elif (count_up_net < 0):
            while(count_up_net != 0):
                pi.set_servo_pulsewidth(pitchpin, forwardp)
                time.sleep(0.02)
                pi.set_servo_pulsewidth(pitchpin, stopp)
                time.sleep(0.4)
                count_up_net += 1
                # print(count_up_net)
        pi.stop()
        return

def main(args=None):
    try:
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

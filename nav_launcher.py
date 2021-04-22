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
pi.write(d1, 0)


THRESHOLDTEMP = 35

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
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (not self.target_hit):
            data = String()
            data = self.detect_target()
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            if (data == 'on target'):
                self.fire_ping_pong()
        return
    
    # identify if there is a target and where it is
    def detect_target(self):
       i = 0
        max_temp = 0
        combined_row = [0 for i in range(8)]
        # store the vertical weights 
        for row in amg.pixels:
            for temp in row:
        #        print(['{0:.1f}'.format(temp)], end = " ")
                if (temp > max_temp):
                    max_temp = temp
                combined_row[i] += temp
         #   print()
            i += 1
        #print(combined_row)
        # compare the vertical weights to identify the target
        weight_left = (combined_row[0] + combined_row[1] + combined_row[2])
        # since we do not stop the bot, we try to fire as soon as the target is in our field of view
        weight_center = (combined_row[3] + combined_row[4] + combined_row[5] + combined_row[6] + combined_row[7]) *3/5
        weight_right = 0
        if (self.target_hit):
            return 'target hit'
        elif (max_temp < 30):
            return 'not found'
        elif (weight_center > weight_left and weight_center > weight_right):
            return 'on target'
        elif (weight_left > weight_center and weight_left > weight_right):
            return 'right'
        elif (weight_right > weight_center):
            return 'left'
        else:
            return '?'

    def fire_ping_pong(self):
        if (not self.target_hit):
            pi.write(en,1)                   # start flywheels
            for i in range (3):
                pi.set_servo_pulsewidth(servo_pin, backward)     # move rack back
                time.sleep(0.21)
                pi.set_servo_pulsewidth(servo_pin, stop)         # stop rack movement
                time.sleep(0.1)
                pi.set_servo_pulsewidth(servo_pin, forward)      # move rack forward to push ball
                time.sleep(0.23)
                pi.set_servo_pulsewidth(servo_pin, stop)         # stop rack movement
                time.sleep(0.1)
                pi.set_servo_pulsewidth(pitchpin, forwardp)      # tilt launcher up
                time.sleep(0.04)
                pi.set_servo_pulsewidth(pitchpin, stopp)
                time.sleep(0.1)
            pi.write(en, 0)                  # stop flywheels
        self.target_hit = True

        pi.set_servo_pulsewidth(pitchpin, backp)                 # reset launcher
        time.sleep(0.14)
        pi.set_servo_pulsewidth(pitchpin, stopp)
        time.sleep(0.4)

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
                                  

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
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO

THRESHOLDTEMP = 32

# setup i2c for ir

GPIO.setmode(GPIO.BCM)
ir_power_pin = 17
GPIO.setup(ir_power_pin, GPIO.OUT)
GPIO.output(ir_power_pin, 1)
time.sleep(1)
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # create publisher for data about target
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.4 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create subscriber to tell launcher when it can fire
        self.launcher_ready_subscription = self.create_subscription(
            String,
            'launcher_ready',
            self.launcher_ready_callback,
            10)

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
        #detect target
        weight_left = combined_row[0] + combined_row[1] + combined_row[2]
        weight_center = (combined_row[3] + combined_row[4]) * 3/2
        weight_right = combined_row[5] + combined_row[6] + combined_row[7]

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
        time.sleep(2)
        if (not self.target_hit):
            # aim picthing
            #pitchcontrols
            forwardp = 6.01
            backp = 7.6
            pitchpin = 18
            stop = 7
            GPIO.setup(pitchpin, GPIO.OUT)
            pp = GPIO.PWM(pitchpin, 50)
            pp.start(7.0)
            combined_col = [0 for i in range(8)]
            middle = False;
            count = 0
            while(not middle and count < 8):
                for row in amg.pixels:
                    col_i = 0;
                    for temp in row:
                        print(['{0:.1f}'.format(temp)], end = " ")
                        combined_col[col_i] += temp
                        col_i += 1;
                    print()
                weight_up = combined_col[0] + combined_col[1] + combined_col[2]
                weight_middle = (combined_col[3] + combined_col[4]) * 3/2
                weight_down = combined_col[5] + combined_col[6] + combined_col[7]

                if (weight_middle > weight_up and weight_middle > weight_down):
                    print("target middle")
                    middle = True
                elif (weight_up > weight_middle and weight_up > weight_down):
                    print("target up")
                    pp.ChangeDutyCycle(backp)
                    time.sleep(0.2)
                    pp.ChangeDutyCycle(stop)
                    time.sleep(0.5)
                elif(weight_down > weight_middle):
                    print("target down")
                    pp.ChangeDutyCycle(forwardp)
                    time.sleep(0.2)
                    pp.ChangeDutyCycle(stop)
                    time.sleep(0.5)
                else:
                    print('not sure')
                count += 1
                time.sleep(0.2)

            en = 21
            in2 = 5
            in1 = 6
            d2 = 19
            d1 = 13
            sf = 16

            #rack controls
            forward = 7.6 #7.6
           backward = 6.6 #6.6
            servo_pin = 4
            GPIO.setup(servo_pin, GPIO.OUT)
            p = GPIO.PWM(servo_pin, 50)
            p.start(7.0)


            GPIO.setup(en, GPIO.OUT)
            GPIO.setup(in2, GPIO.OUT)
            GPIO.setup(in1, GPIO.OUT)
            GPIO.setup(d2, GPIO.OUT)
            GPIO.setup(d1, GPIO.OUT)

            GPIO.output(in2, 0)
            GPIO.output(in1, 1)
            GPIO.output(d2, 1)
            GPIO.output(d1, 0)

            #GPIO.output(en,1)                   # start flywheels
            for i in range (2):
                time.sleep(1)
                p.ChangeDutyCycle(forward)      # move rack forward
                #0.23
                time.sleep(0.23)
                p.ChangeDutyCycle(stop)         # stop rack movement
                time.sleep(1)
                p.ChangeDutyCycle(backward)     # move rac            GPIO.cleanup()
        self.target_hit = True
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
        #GPIO.output(en,0)
        #GPIO.cleanup()
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
k back to load next ball
                #0.23
                time.sleep(0.21)
                p.ChangeDutyCycle(stop)         # stop rack movement
                time.sleep(1.5)
            GPIO.output(en, 0)                  # stop flywheels
            GPIO.cleanup()
        self.target_hit = True
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
        #GPIO.output(en,0)
            GPIO.cleanup()
        self.target_hit = True
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
        #GPIO.output(en,0)
        #GPIO.cleanup()
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

                time.sleep(0.5)
            pi.write(en, 0)                  # stop flywheels
        self.target_hit = True

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
        #GPIO.output(en,0)
        #GPIO.cleanup()
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

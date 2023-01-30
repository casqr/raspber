import RPi.GPIO as GPIO
import time
import math
from geometry_msgs.msg import Twist
import rospy


class DifferentialDriveController:
    def __init__(self):
        # Get parameters from the parameter server
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.066)
        self.wheel_base = rospy.get_param("~wheel_base", 0.133)
        self.max_pwm = rospy.get_param("~max_pwm", 100)
        self.min_pwm = rospy.get_param("~min_pwm", 0)

        self.pin_left_forward = 26
        self.pin_left_backward = 13
        self.pin_left_pwm = 5
        self.pin_right_forward = 21
        self.pin_right_backward = 16
        self.pin_right_pwm = 12

        self.left_motor_speed = 0
        self.right_motor_speed = 0

        # Set up GPIO pins for the L298N motor controller
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_left_forward, GPIO.OUT)
        GPIO.setup(self.pin_left_backward, GPIO.OUT)
        GPIO.setup(self.pin_left_pwm, GPIO.OUT)
        GPIO.setup(self.pin_right_forward, GPIO.OUT)
        GPIO.setup(self.pin_right_backward, GPIO.OUT)
        GPIO.setup(self.pin_right_pwm, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.pin_left_pwm, 100)
        self.right_pwm = GPIO.PWM(self.pin_right_pwm, 100)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def twist_callback(self, data):
        linear_x = data.linear.x
        angular_z = data.angular.z

        # calculate the left and right motor speeds based on the linear and angular velocities
        left_speed = (2 * linear_x - angular_z * self.wheel_base) / (2 * self.wheel_diameter)
        right_speed = (2 * linear_x + angular_z * self.wheel_base) / (2 * self.wheel_diameter)
        
        rospy.loginfo(f"Left speed: {left_speed}\nRight speed: {right_speed}")

        self.set_left_motor_speed(left_speed)
        self.set_right_motor_speed(right_speed)

    def set_left_motor_speed(self, speed):
        if speed > 0:
            GPIO.output(self.pin_left_forward, 1)
            GPIO.output(self.pin_left_backward, 0)
            self.left_pwm.ChangeDutyCycle(min(speed, self.max_pwm))
        else:
            GPIO.output(self.pin_left_forward, 0)
            GPIO.output(self.pin_left_backward, 1)
            self.left_pwm.ChangeDutyCycle(max(abs(speed), self.min_pwm))

    def set_right_motor_speed(self, speed):
        if speed > 0:
            GPIO.output(self.pin_right_forward, 1)
            GPIO.output(self.pin_right_backward, 0)
            self.right_pwm.ChangeDutyCycle(min(speed, self.max_pwm))
        else:
            GPIO.output(self.pin_right_forward, 0)
            GPIO.output(self.pin_right_backward, 1)
            self.right_pwm.ChangeDutyCycle(max(abs(speed), self.min_pwm))

    def run(self):
        # Initialize ROS node
        rospy.init_node('differential_drive_controller')
        rospy.loginfo("Node started")

        # Subscribe to Twist messages
        rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

        # Spin to keep the script running
        rospy.spin()

    def __del__(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    try:
        controller = DifferentialDriveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
import tf


class OdometryPoseController:
    def __init__(self):
        # Set up photo interrupter sensor for encoder
        self.pin_left_encoder = 12
        self.pin_right_encoder = 16

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_left_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_right_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.pin_left_encoder, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.pin_right_encoder, GPIO.RISING, callback=self.right_encoder_callback)

        self.encoder_pulses_per_rev = 20
        self.wheel_diameter = 0.1  # in meters
        self.wheel_base = 0.2  # in meters
        self.x = 0
        self.y = 0
        self.theta = 0
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_time = rospy.Time.now()
        self.x_vel = 0
        self.rot_vel = 0
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel):
        self.right_encoder_count += 1

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        #
        left_delta = self.left_encoder_count / self.encoder_pulses_per_rev * self.wheel_diameter * math.pi
        right_delta = self.right_encoder_count / self.encoder_pulses_per_rev * self.wheel_diameter * math.pi

        # reset the encoder count
        self.left_encoder_count = 0
        self.right_encoder_count = 0

        distance = (left_delta + right_delta) / 2
        delta_theta = (right_delta - left_delta) / self.wheel_base
        self.x += distance * math.cos(self.theta + delta_theta / 2)
        self.y += distance * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta
        # Calculate linear and angular velocity
        self.x_vel = distance / dt
        self.rot_vel = delta_theta / dt

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.x_vel
        odom.twist.twist.angular.z = self.rot_vel
        self.odom_publisher.publish(odom)

    def run(self):
        # Initialize ROS node
        rospy.init_node('differential_drive_controller')

        # Set up rate for odometry update
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_odometry()
            rate.sleep()


if __name__ == '__main__':
    controller = OdometryPoseController()
    controller.run()
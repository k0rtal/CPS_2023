#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


TTC_PARAMETER = 4.12
DECELERATION_OF_F1TENTH_CAR = 7.26


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        # TODO: create ROS subscribers and publishers.
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.publish_bool = self.create_publisher(Bool, '/brake_bool', 1)
        self.publish_drive = self.create_publisher(AckermannDriveStamped, '/drive', 1)

        self.speed = 0.
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        size_of_scan_ranges = len(scan_msg.ranges)
        for i in range(size_of_scan_ranges):
            range_ = scan_msg.ranges[i]
            # check LiDAR range
            if range_ > scan_msg.range_max or range_ < scan_msg.range_min:
                continue

            angle_to_heading = scan_msg.angle_min + i * scan_msg.angle_increment
            projected_speed = max(-1 * self.speed * np.cos(angle_to_heading), 0)

            ttc = float('inf') if projected_speed == 0 else range_ / projected_speed

            # TODO: publish command to brake
            # publish command to brake
            if ttc <= TTC_PARAMETER * self.speed / DECELERATION_OF_F1TENTH_CAR:
                # send brake message
                msg_brake = AckermannDriveStamped()
                msg_brake.drive = AckermannDrive(speed=0.0)
                self.publish_drive.publish(msg_brake)

                # send brake bool message
                msg_bool = Bool()
                msg_bool.data = True
                self.publish_bool.publish(msg_bool)
                self.get_logger().info('Emergency brake')


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

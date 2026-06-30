import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .geometry import (
    angle_diff_rad,
    point_to_point_distance,
    point_to_point_heading,
    quaternion_to_yaw,
)


class RobotNav(Node):
    def __init__(self):
        super().__init__("robot_nav")

        self.scan_sub = self.create_subscription(
            Odometry, "odom", self.nav_callback, qos_profile=qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.last_print_time = self.get_clock().now()
        self.distance_traveled_m = 0.0
        self.current_state = Navigation.TURNING
        self.waypoint = Point(x=3.0, y=5.0, z=0.0)

    def nav_callback(self, msg: Odometry):
        if self.checkpoint_m_m is None:
            self.checkpoint_m_m = msg.pose.pose.position
        if self.checkpoint_rad is None:
            self.checkpoint_rad = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.current_state == Navigation.DRIVING:
            if self.drive_forward(odom_sub=msg, distance_m=2.5, speed_mps=0.75):
                self.checkpoint_rad = quaternion_to_yaw(msg.pose.pose.orientation)
                self.current_state = Navigation.OBSTACLE
        if self.current_state == Navigation.OBSTACLE:
            if self.turn(odom_sub=msg, angle_rad=(math.pi / 2), speed_radps=0.6):
                self.checkpoint_m_m = msg.pose.pose.position
                self.current_state = Navigation.DRIVING

    def test_callback(self, msg):
        print(f"Angle: {quaternion_to_yaw(msg.pose.pose.orientation):.2f}")

    def drive_forward(self, *, scan_sub: LaserScan, speed_mps: float):
        cmd = TwistStamped()

        front_sector = (
            scan_sub.ranges[360 - 10 // 2 : 360] + scan_sub.ranges[0 : 10 // 2]
        )
        valid_front_ranges = [r for r in front_sector if math.isfinite(r)]

        if valid_front_ranges:
            front_distance = min(valid_front_ranges)
        else:
            front_distance = float("inf")

        if front_distance < 1.5:
            cmd.twist.x = 0.0
            return True
        else:
            cmd.twist.x = speed_mps

        self.cmd_pub.publish(cmd)

    def turn(self, *, odom_sub: Odometry, angle_rad: float, speed_radps: float):
        cmd = TwistStamped()
        yaw = quaternion_to_yaw(odom_sub.pose.pose.orientation)
        if abs(angle_diff_rad(yaw, self.checkpoint_rad)) < angle_rad:
            cmd.twist.angular.z = speed_radps
        else:
            cmd.twist.angular.z = 0.0
            return True
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)


class Navigation(Enum):
    OBSTACLE = 0
    DRIVING = 1
    TURNING = 2
    FINISHED = 3


def main(args=None):
    rclpy.init(args=args)

    odom_driver = RobotNav()

    rclpy.spin(odom_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

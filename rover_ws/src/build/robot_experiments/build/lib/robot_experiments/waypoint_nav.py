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

        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, qos_profile=qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )
        timer_period_s = 0.05
        self.timer_callback = self.create_timer(timer_period_s, self.control_loop)
        self.odom_msg = None
        self.scan_msg = None
        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.last_print_time = self.get_clock().now()
        self.distance_traveled_m = 0.0
        self.checkpint_rad = None
        self.current_state = Navigation.TURNING
        self.waypoint = Point(x=3.0, y=5.0, z=0.0)
        self.p_rad = 3.0

    def control_loop(self):
        if self.scan_msg is None or self.odom_msg is None:
            return
        if self.checkpoint_m_m is None:
            self.checkpoint_m_m = self.odom_msg.pose.pose.position
        if self.checkpoint_rad is None:
            self.checkpoint_rad = quaternion_to_yaw(self.odom_msg.pose.pose.orientation)

        front_ranges = (
            self.scan_msg.ranges[360 - 10 // 2 : 360]
            + self.scan_msg.ranges[0 : 10 // 2]
        )
        front_sector = [r for r in front_ranges if math.isfinite(r)]
        if front_sector:
            front_distance = min(front_sector)
        else:
            front_distance = float("inf")

        if self.current_state == Navigation.FINISHED:
            self.stop()
        if self.current_state == Navigation.DRIVING:
            if self.drive_forward(front_scan=front_distance, speed_mps=0.75):
                self.current_state = Navigation.OBSTACLE
            elif (
                point_to_point_distance(self.odom_msg.pose.pose.position, self.waypoint)
                < 0.2
            ):
                self.current_state = Navigation.FINISHED
        if self.current_state == Navigation.OBSTACLE:
            if self.avoid_obstacle(front_scan=front_distance, speed_radps=0.6):
                self.current_state = Navigation.TURNING
        if self.current_state == Navigation.TURNING:
            if self.turn_to_waypoint():
                self.current_state = Navigation.DRIVING

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def test_callback(self, msg):
        print(f"Angle: {quaternion_to_yaw(msg.pose.pose.orientation):.2f}")

    def drive_forward(self, *, front_scan: float, speed_mps: float):
        cmd = TwistStamped()

        if front_scan < 1.5:
            cmd.twist.linear.x = 0.0
            return True
        else:
            cmd.twist.linear.x = speed_mps
        self.cmd_pub.publish(cmd)

    def avoid_obstacle(self, *, front_scan: float, speed_radps: float):
        cmd = TwistStamped()
        if front_scan >= 1.5:
            cmd.twist.angular.z = 0.0
            return True
        else:
            cmd.twist.angular.z = speed_radps
        self.cmd_pub.publish(cmd)

    def turn_to_waypoint(self):
        cmd = TwistStamped()
        error_rad = point_to_point_heading(
            self.odom_msg.pose.pose.position, self.waypoint
        )
        if error_rad < 0.1:
            cmd.twist.angular.z = 0.0
            return True
        else:
            cmd.twist.angular.z = self.p_rad * error_rad
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

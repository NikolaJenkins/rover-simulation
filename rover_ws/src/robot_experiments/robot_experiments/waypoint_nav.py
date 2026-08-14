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
    point_to_point_distance_m,
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
        self.timer_callback = self.create_timer(0.05, self.control_loop)
        self.odom_msg = None
        self.scan_msg = None
        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.obstacle_threshold_m = 0.75
        self.detector_sector_range = 20
        self.current_state = Navigation.TURNING
        self.waypoint = Point(x=2.0, y=3.0, z=0.0)
        self.distance_to_waypoint_m: float = float("inf")
        self.kp_rad = 1.0

    def control_loop(self):
        if self.scan_msg is None or self.odom_msg is None:
            return
        if not math.isfinite(self.distance_to_waypoint_m):
            self.distance_to_waypoint_m = point_to_point_distance_m(
                point1=self.odom_msg.pose.pose.position, point2=self.waypoint
            )

        front_ranges = (
            self.scan_msg.ranges[360 - self.detector_sector_range // 2 : 360]
            + self.scan_msg.ranges[0 : self.detector_sector_range // 2]
        )
        front_sector = [r for r in front_ranges if math.isfinite(r)]
        if front_sector:
            front_distance = min(front_sector)
        else:
            front_distance = float("inf")
        left_ranges = self.scan_msg.ranges[
            270 - self.detector_sector_range // 2 : 270
            + self.detector_sector_range // 2
        ]
        left_sector = [r for r in left_ranges if math.isfinite(r)]
        if left_sector:
            left_distance = min(left_sector)
        else:
            left_distance = float("inf")
        right_ranges = self.scan_msg.ranges[
            90 - self.detector_sector_range // 2 : 90 + self.detector_sector_range // 2
        ]
        right_sector = [r for r in right_ranges if math.isfinite(r)]
        if right_sector:
            right_distance = min(left_sector)
        else:
            right_distance = float("inf")

        if self.current_state == Navigation.DRIVING:
            self.distance_to_waypoint_m = point_to_point_distance_m(
                point1=self.odom_msg.pose.pose.position, point2=self.waypoint
            )
            if self.distance_to_waypoint_m < 0.1:
                self.stop()
                self.current_state = Navigation.FINISHED
            match self.drive_forward(
                front_scan=front_distance,
                left_scan=left_distance,
                right_scan=right_distance,
                speed_mps=0.6,
            ):
                case True:
                    self.current_state = Navigation.OBSTACLE
                case False:
                    self.current_state = Navigation.TURNING
        if self.current_state == Navigation.OBSTACLE:
            self.distance_to_waypoint_m = point_to_point_distance_m(
                point1=self.odom_msg.pose.pose.position, point2=self.waypoint
            )
            if self.distance_to_waypoint_m < 0.1:
                self.stop()
                self.current_state = Navigation.FINISHED
            match self.avoid_obstacle(
                front_scan=front_distance,
                left_scan=left_distance,
                right_scan=right_distance,
            ):
                case True:
                    self.current_state = Navigation.DRIVING
                case False:
                    self.current_state = Navigation.TURNING

        if self.current_state == Navigation.TURNING:
            if self.turn_to_waypoint():
                self.current_state = Navigation.DRIVING
        print(f"Current state: {self.current_state.name}")
        print(f"x: {self.odom_msg.pose.pose.position.x:.2f}")
        print(f"y: {self.odom_msg.pose.pose.position.y:.2f}")

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def test_callback(self, msg):
        print(f"Angle: {quaternion_to_yaw(q=msg.pose.pose.orientation):.2f}")

    def drive_forward(
        self,
        *,
        front_scan: float,
        left_scan: float,
        right_scan: float,
        speed_mps: float = 0.75,
    ):
        cmd = TwistStamped()

        if front_scan < self.obstacle_threshold_m:
            cmd.twist.linear.x = 0.0
            return True
        else:
            if (
                (
                    point_to_point_distance_m(
                        point1=self.odom_msg.pose.pose.position, point2=self.waypoint
                    )
                    > self.distance_to_waypoint_m
                )
                and left_scan >= 1.0
                and right_scan >= 1.0
            ):
                cmd.twist.linear.x = 0.0
                return False
            cmd.twist.linear.x = speed_mps
        self.cmd_pub.publish(cmd)

    def avoid_obstacle(
        self,
        *,
        front_scan: float,
        left_scan: float,
        right_scan: float,
        speed_radps: float = 0.6,
    ):
        cmd = TwistStamped()
        if front_scan >= self.obstacle_threshold_m:
            cmd.twist.angular.z = 0.0
            if left_scan < 1.0 or right_scan < 1.0:
                # set to DRIVING
                return True
            else:
                # set to TURNING
                return False
        else:
            if left_scan >= right_scan:
                cmd.twist.angular.z = speed_radps
            else:
                cmd.twist.angular.z = -speed_radps
            # cmd.twist.linear.x = 0.2
        self.cmd_pub.publish(cmd)

    def turn_to_waypoint(self):
        cmd = TwistStamped()
        target_angle_rad = point_to_point_heading(
            point1=self.odom_msg.pose.pose.position, point2=self.waypoint
        )
        actual_angle_rad = quaternion_to_yaw(q=self.odom_msg.pose.pose.orientation)
        error_rad = angle_diff_rad(a_rad=actual_angle_rad, b_rad=target_angle_rad)
        if abs(error_rad) < 0.01:
            cmd.twist.angular.z = 0.0
            return True
        else:
            cmd.twist.angular.z = self.kp_rad * error_rad
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def clear_path_exists(self) -> bool:
        target_heading = point_to_point_heading(
            point1=self.odom_msg.pose.pose.position, point2=self.waypoint
        )
        heading_diff = int(
            angle_diff_rad(
                a_rad=self.odom_msg.pose.pose.orientation.z, b_rad=target_heading
            )
        )
        target_ranges = self.scan_msg.ranges[heading_diff - 5 : heading_diff + 5]
        target_sector = [r for r in target_ranges if math.isfinite(r)]
        if target_sector:
            target_range = min(target_sector)
        else:
            target_range = float("inf")
        return self.scan_msg.ranges[target_range] >= point_to_point_distance_m(
            point1=self.odom_msg.pose.pose.position, point2=self.waypoint
        )


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

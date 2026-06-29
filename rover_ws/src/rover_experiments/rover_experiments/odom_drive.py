import math
from enum import Enum

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .geometry import angle_diff_rad, point_distance, quaternion_to_yaw


class RobotOdom(Node):
    def __init__(self):
        super().__init__("robot_scan")

        self.scan_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, qos_profile=qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.last_print_time = self.get_clock().now()
        self.checkpoint_m_m = None
        self.checkpoint_rad = None
        self.distance_traveled_m = 0.0
        self.current_state = DriveState.DRIVING
        self.side = 0

    def odom_callback(self, msg: Odometry):
        if self.checkpoint_m_m is None:
            self.checkpoint_m_m = msg.pose.pose.position
        if self.checkpoint_rad is None:
            self.checkpoint_rad = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.side == 4:
            self.stop()
            return
        if self.current_state == DriveState.DRIVING:
            if self.drive_forward(odom_sub=msg, distance_m=2.5, speed_mps=0.75):
                self.checkpoint_rad = quaternion_to_yaw(msg.pose.pose.orientation)
                self.current_state = DriveState.TURNING
        if self.current_state == DriveState.TURNING:
            if self.turn(odom_sub=msg, angle_rad=(math.pi / 2), speed_radps=0.6):
                self.checkpoint_m_m = msg.pose.pose.position
                self.current_state = DriveState.DRIVING
                self.side += 1

    def test_callback(self, msg):
        print(f"Angle: {quaternion_to_yaw(msg.pose.pose.orientation):.2f}")

    def drive_forward(self, *, odom_sub: Odometry, distance_m: float, speed_mps: float):
        cmd = TwistStamped()
        if (
            point_distance(self.checkpoint_m_m, odom_sub.pose.pose.position)
            < distance_m
        ):
            cmd.twist.linear.x = speed_mps
        else:
            cmd.twist.linear.x = 0.0
            return True
        self.cmd_pub.publish(cmd)

    # TODO: robot can only turn once
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


class DriveState(Enum):
    DRIVING = 0
    TURNING = 1


def main(args=None):
    rclpy.init(args=args)

    odom_driver = RobotOdom()

    rclpy.spin(odom_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

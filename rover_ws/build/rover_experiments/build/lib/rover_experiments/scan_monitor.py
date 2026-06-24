import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class RobotScan(Node):
    def __init__(self):
        super().__init__("robot_scan")

        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.angle_range = 20

    def scan_callback(self, msg):
        cmd = TwistStamped()
        front_sector = (
            msg.ranges[360 - self.angle_range // 2 : 360]
            + msg.ranges[0 : self.angle_range // 2]
        )
        valid_front_ranges = [r for r in front_sector if math.isfinite(r)]
        if valid_front_ranges:
            front_distance = min(valid_front_ranges)
        else:
            front_distance = float("inf")

        left_sector = msg.ranges[
            270 - self.angle_range // 2 : 270 + self.angle_range // 2
        ]
        valid_left_ranges = [r for r in left_sector if math.isfinite(r)]
        if valid_left_ranges:
            left_distance = min(valid_left_ranges)
        else:
            left_distance = float("inf")

        right_sector = msg.ranges[
            90 - self.angle_range // 2 : 90 + self.angle_range // 2
        ]
        valid_right_ranges = [r for r in right_sector if math.isfinite(r)]
        if valid_right_ranges:
            right_distance = min(valid_right_ranges)
        else:
            right_distance = float("inf")

        if front_distance < 0.75:
            print("TURN")
            if left_distance > right_distance:
                cmd.twist.angular.z = -0.2
            else:
                cmd.twist.angular.z = 0.2
        else:
            print("FORWARD")
            cmd.twist.linear.x = 0.3

        print("Front obstacle:", front_distance)
        self.cmd_pub.publish(cmd)

    def test_callback(self, msg):
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.5
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    obstacle_detector = RobotScan()

    rclpy.spin(obstacle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

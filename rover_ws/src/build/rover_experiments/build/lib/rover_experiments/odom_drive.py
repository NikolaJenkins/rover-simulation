import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class RobotOdom(Node):
    def __init__(self):
        super().__init__("robot_scan")

        self.scan_sub = self.create_subscription(
            Odometry, "odom", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.last_print_time = self.get_clock().now()

    def scan_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds < 1e9:
            return
        self.last_print_time = now
        print(f"x: {msg.pose.pose.position.x:.2f}")
        print(f"y: {msg.pose.pose.position.y:.2f}")
        print(f"heading: {msg.pose.pose.orientation.z:.2f}")

    def test_callback(self, msg):
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.5
        self.cmd_pub.publish(cmd)


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

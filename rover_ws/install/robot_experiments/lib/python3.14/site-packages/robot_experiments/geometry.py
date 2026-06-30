import math

from geometry_msgs.msg import Point, Quaternion


def point_distance(point1: Point, point2: Point) -> float:
    return math.sqrt(
        (point1.x - point2.x) ** 2
        + (point1.y - point2.y) ** 2
        + (point1.z - point2.z) ** 2
    )


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2),
    )


def angle_diff_rad(a_rad: float, b_rad: float) -> float:
    return math.atan2(
        math.sin(a_rad - b_rad),
        math.cos(a_rad - b_rad),
    )

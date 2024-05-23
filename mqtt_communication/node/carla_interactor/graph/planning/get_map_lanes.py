import math
import numpy as np
from scipy.spatial.transform import Rotation as R

from .monitor_classes import Node3D


def get_lanes_points(waypoint_list, rgb=[0, 0, 0]):
    """
    Get markers to represent map topology defined by every lane.

    Args:
        waypoint_list: List of waypoints centered in every lane of the map
            at a given distance.

    Returns:
        points: List of points defining every lane of the map
    """
    map_points = []

    for waypoint in waypoint_list:
        yaw = math.radians(waypoint.transform.rotation.yaw)
        distance = waypoint.lane_width / 2
        k = 1 if waypoint.lane_id < 0 else -1

        node_r = Node3D(
            x=waypoint.transform.location.x + math.cos(yaw) * distance * (-k),
            y=waypoint.transform.location.y - math.sin(yaw) * distance * k,
            z=waypoint.transform.location.z
        )

        node_l = Node3D(
            x=waypoint.transform.location.x - math.cos(yaw) * distance * (-k),
            y=waypoint.transform.location.y + math.sin(yaw) * distance * k,
            z=waypoint.transform.location.z
        )

        map_points.extend([node_r, node_l])

    return map_points


def euler_to_quaternion(roll, pitch, yaw):
    """
    Return the orientation of our ego-vehicle in quaternion based on Euler angles (Roll = x, Pitch = y, Yaw = z)
    """
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    q = r.as_quat()  # returns in the order [qx, qy, qz, qw]
    return Quaternion(q[0], q[1], q[2], q[3])

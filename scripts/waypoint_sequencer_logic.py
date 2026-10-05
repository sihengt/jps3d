#!/usr/bin/env python3
"""Pure decision logic for sequencing through a list of waypoints.

Kept free of ROS/rclpy so it can be unit tested directly (mirrors the
compute_decel_vel_max pattern in droan_gl/velocity_ramp.hpp).
"""
import math


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def compute_next_target(current_index, position, waypoints, switch_radius):
    """Decide which waypoint index to target next.

    Returns the waypoint index that should be commanded, or None if the
    mission is complete (the drone has reached the switch radius of the
    final waypoint).
    """
    if current_index >= len(waypoints):
        return None

    if distance(position, waypoints[current_index]) < switch_radius:
        next_index = current_index + 1
        if next_index >= len(waypoints):
            return None
        return next_index

    return current_index

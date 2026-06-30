#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from waypoint_sequencer_logic import compute_next_target  # noqa: E402

WAYPOINTS = [(0.0, 0.0, 1.0), (5.0, 0.0, 1.0), (5.0, 5.0, 1.0)]


def test_far_from_current_waypoint_stays_on_same_index():
    position = (0.0, 0.0, 5.0)  # 4m away from waypoint 0
    assert compute_next_target(0, position, WAYPOINTS, switch_radius=0.5) == 0


def test_within_radius_of_non_last_waypoint_advances_to_next():
    position = (0.1, 0.0, 1.0)  # 0.1m from waypoint 0
    assert compute_next_target(0, position, WAYPOINTS, switch_radius=0.5) == 1


def test_within_radius_of_last_waypoint_completes_mission():
    position = (5.0, 5.1, 1.0)  # 0.1m from waypoint 2 (last)
    assert compute_next_target(2, position, WAYPOINTS, switch_radius=0.5) is None


def test_exactly_at_switch_radius_does_not_switch():
    # distance == switch_radius should NOT trigger a switch (strict '<', matching
    # the seen_radius convention used elsewhere in this codebase).
    position = (0.5, 0.0, 1.0)  # exactly 0.5m from waypoint 0
    assert compute_next_target(0, position, WAYPOINTS, switch_radius=0.5) == 0


def test_already_complete_mission_stays_complete():
    position = (5.0, 5.0, 1.0)
    assert compute_next_target(len(WAYPOINTS), position, WAYPOINTS, switch_radius=0.5) is None

#!/usr/bin/env python3
GOAL_STATE = [
    -200,
    200,
    50
]

WAYPOINTS = [
    [0.0, 0.0, 10.0], 
    [0.0, 50.0, 10.0], 
    [-30.0, 0.0, 10.0],
    [0.0, 0.0, 10.0],
    ]


# how close drone should be to waypoint before going to next
WAY_PROXIMITY = 8.0
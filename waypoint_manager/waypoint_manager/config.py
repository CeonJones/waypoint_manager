#!/usr/bin/env python3
GOAL_STATE = [
    -200,
    200,
    50
]

WAYPOINTS = [ 
    [-100.0, 150.0, 55.0], 
    [-100.0, -200.0, 55.0],
    ]

# how close drone should be to waypoint before going to next
WAY_PROXIMITY = 15.0

# boundary drone cant go outside of
MAX_RANGE = 80

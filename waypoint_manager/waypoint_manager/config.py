#!/usr/bin/env python3
GOAL_STATE = [
    -200,
    200,
    50
]

WAYPOINTS = [
    [500.0, 0.0, 10.0], 
    [0.0, 950.0, 15.0], 
    [-500.0, 50.0, 12.0]
    ]


# how close drone should be to waypoint before going to next
WAY_PROXIMITY = 20.0
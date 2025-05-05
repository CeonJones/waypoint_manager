#!/usr/bin/env python3
import rclpy
import math
import numpy as np


from rclpy.node import Node
from rclpy.duration import Duration
from typing import List
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from nav_msgs.msg import Odometry
from waypoint_manager.config import GOAL_STATE, WAY_PROXIMITY, WAYPOINTS
from rclpy.timer import Rate
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy

class WaypointManager(Node):
    """
    Establishes waypoints for drone nav which is then  given to the target
    estimator node to be used for its trajectory
    """
    
    def __init__ (self, ns=''):
        super().__init__('waypoint_manager')

        # publshes odemetry messages that that guidance publisher will subscribe to
        # https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        self.target_publisher: Publisher = self.create_publisher(
            Odometry, 'target_position', 10)

        # ENU waypoints
        self.waypoint_list = WAYPOINTS

        
        # set waypoint index 
        self.current_waypoint_index: int = 0
        self.current_position: List[float] = None
        #self.origin_position: List[float] = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT       
        
        # subscribes to the mavros local position odometry topic
        # https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        self.position_subscriber: Subscription = self.create_subscription(
            Odometry, 'mavros/local_position/odom', self.drone_position_callback, qos)
        
        # heartbeat for the node to check its proximity to waypoints
        self.timer_period: float = 0.05
        self.timer = self.create_timer(
            self.timer_period, self.update_waypoint)
            

    def drone_position_callback(self, msg: Odometry) -> None:
        """
        Callback for the drone position that is being publised by odometry publisher
        """
        self.current_position = [ msg.pose.pose.position.x,  msg.pose.pose.position.y,  msg.pose.pose.position.z]

        
        #print(f"Current position: {self.current_position}")
    def update_waypoint(self) -> None:
        """S
        Check proximity, switch waypoints, and pubish to guidance publisher
        """
        if self.current_position is None:
            return
        
        home_distance =  np.linalg.norm(np.array(self.current_position[:2]))
        
        if home_distance > MAX_RANGE:
            print(f"drone out of bounds")
            msg = Odometry()
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 10.0
            self.target_publisher.publish(msg)
            return
        
        # check distance from current position to waypoint
        dist = np.linalg.norm(np.array(self.current_position) - np.array(self.waypoint_list[self.current_waypoint_index]))
        print(f"Current wayppoint: {self.waypoint_list[self.current_waypoint_index]} and distance: {dist}")
       
        if dist <= WAY_PROXIMITY:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoint_list):
                return
        self.publish_target()

    def publish_target(self) -> None:
        """
        publish way point to target position
        """
        msg = Odometry()
        msg.pose.pose.position.x = self.waypoint_list[self.current_waypoint_index][0]
        msg.pose.pose.position.y = self.waypoint_list[self.current_waypoint_index][1]
        msg.pose.pose.position.z = self.waypoint_list[self.current_waypoint_index][2]
        self.target_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()

    while rclpy.ok():
        try:
            rclpy.spin_once(waypoint_manager, timeout_sec=0.1)

        except KeyboardInterrupt:
            break

    waypoint_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()








        


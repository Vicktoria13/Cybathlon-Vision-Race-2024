import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Direction(Node):
    def __init__(self):
        super().__init__('direction')

        self.pose_subscription = self.create_subscription(
            String,
            '/string_dijkstra',
            self.pose_callback,
            1)
        
        
        self.publisher_ = self.create_publisher(
            Point,
            'direction',
            1)


    def pose_callback(self, msg):
        msgP = Point()
        #self.get_logger().info('pose callback start')
        self.pose = msg.data
        if self.pose == '9h':
                self.x = -4.0
                self.y = 14.0
        elif self.pose == '10h30':
            self.x = -3.5
            self.y = 18.0
        elif self.pose == '12h':
            self.x = 0.0
            self.y = 20.0
        elif self.pose == '1h30':
            self.x = 3.5
            self.y = 18.0
        elif self.pose == '3h':
            self.x = 4.0
            self.y = 14.0
        else:
            self.x = 0.0
            self.y = 15.0
        msgP.x = self.x
        msgP.y = self.y
        self.publisher_.publish(msgP)
        self.get_logger().info(f'Publishing: {msgP.x}, {msgP.y}')

        


def main(args=None):
    rclpy.init(args=args)
    dks_publisher = Direction()
    rclpy.spin(dks_publisher)
    dks_publisher.destroy_node()
    rclpy.shutdown()
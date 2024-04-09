#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import GetPath
from AStar_python_interface import A_Star, CSpace
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from PurePursuit_python_interface import Trajectory, PurePersuit_Controller

# Subsciber
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathTracking_Subscriber(Node):
    def __init__(self):
        super().__init__('Path_Tracking_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def amcl_pose_callback(self, pose: PoseWithCovarianceStamped):
        x_pos = pose.pose.pose.position.x
        y_pos = pose.pose.pose.position.y


        


def main(args=None):
    rclpy.init(args=args)
    path_tracking_sub = PathTracking_Subscriber()
    rclpy.spin(path_tracking_sub)
    path_tracking_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
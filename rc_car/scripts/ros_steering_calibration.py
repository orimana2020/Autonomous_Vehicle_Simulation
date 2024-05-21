#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import  TransformStamped
from py_PurePursuit import PurePersuit_Controller
from py_Utils import euler_from_quaternion, Trajectory, State
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.parameter import Parameter
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class PathTracking(Node):
    def __init__(self):
        super().__init__('Path_Tracking')
        #  hyper-parameters
        k = 0.1  # look forward gain
        Lfc = 0.8  # [m] look-ahead distance
        Kp = 1.0  # speed proportional gain
        self.TargetSpeed = 1.0  # [m/s]
        self.MAX_STEER = np.deg2rad(35.0)  # maximum steering angle [rad]
        MAX_DSTEER = np.deg2rad(150.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = 2.0 # maximum speed [m/s]
        self.MIN_SPEED = 1.0  # minimum speed [m/s]
        MAX_ACCEL = 1.0  # maximum accel [m/ss]
        self.wheel_radius = 0.056
        self.WB = 0.335
 
        self.control_command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_control_cmd)
        # self.prev_time = self.get_clock().now()
        
        
    
    def publish_control_cmd(self):
        rear_wheel_angular_speed = 0.7 / self.wheel_radius
        steering_angle = self.MAX_STEER
        cmd_vel = Twist()
        cmd_vel.linear.x = rear_wheel_angular_speed * self.wheel_radius * np.cos(steering_angle) 
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = cmd_vel.linear.x * np.tan(steering_angle) / self.WB
        self.control_command_publisher.publish(cmd_vel)
       

    
def main(args=None):
    rclpy.init(args=args)
    path_tracking_sub = PathTracking()
    rclpy.spin(path_tracking_sub)
    path_tracking_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
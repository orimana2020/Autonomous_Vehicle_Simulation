#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import GetPath
from AStar_python_interface import A_Star, CSpace
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from PurePursuit_python_interface import PurePersuit_Controller
from utils import euler_from_quaternion, Trajectory, State
from geometry_msgs.msg import Twist


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
        self.subscription  

        #  hyper-parameters
        k = 0.1  # look forward gain
        Lfc = 0.5  # [m] look-ahead distance
        Kp = 1.0  # speed proportional gain
        self.TargetSpeed = 2  # [m/s]
        MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
        MAX_DSTEER = np.deg2rad(90.0)  # maximum steering speed [rad/s]
        MAX_SPEED = 10.0 / 3.6  # maximum speed [m/s]
        MIN_SPEED = 0.1  # minimum speed [m/s]
        MAX_ACCEL = 1.0  # maximum accel [m/ss]
        self.wheel_radius = 0.056
        self.WB = 0.3429
        self.max_radial_velocity_rear_wheel = MAX_SPEED / self.wheel_radius

        path = np.load('path_index.npy')
        self.trajectory = Trajectory(dl=0.5, path =path, TARGET_SPEED=self.TargetSpeed)
        self.state = State(WB=self.WB, x=self.trajectory.cx[0], y=self.trajectory.cy[0], yaw=self.trajectory.cyaw[0], v=0.0)
        self.lastIndex = len(self.trajectory.cx) - 1
        self.pp = PurePersuit_Controller(self.trajectory.cx, self.trajectory.cy, k, Lfc, Kp, self.WB, MAX_ACCEL, MAX_SPEED, MIN_SPEED, MAX_STEER, MAX_DSTEER)
        self.target_ind, _ = self.pp.search_target_index(self.state)
        self.control_command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        self.pure_persuit_frrquency = 2


    def amcl_pose_callback(self, pose: PoseWithCovarianceStamped):
        self.update_state(pose)
        delta_t = self.update_time()
        if self.target_ind < self.lastIndex and (delta_t > 1 / self.pure_persuit_frrquency):
            self.state.v = self.pp.proportional_control_acceleration(self.TargetSpeed, self.state.v)
            delta, self.target_ind = self.pp.pure_pursuit_steer_control(self.state, self.trajectory, self.target_ind, dt)
            self.state.predelta = delta
            self.publish_control_cmd(steering_angle=delta, linear_speed = self.state.v)
            self.prev_time = self.get_clock().now()


    
    def update_state(self, pose: PoseWithCovarianceStamped):
        x = self.state.x = pose.pose.pose.position.x
        y = self.state.y = pose.pose.pose.position.y
        _, _, yaw = euler_from_quaternion(pose.pose.pose.orientation)
        self.state.update(x, y, yaw)

    def update_time(self):
        delta_t = (self.get_clock().now() - self.prev_time).to_msg()
        delta_t = delta_t.sec + delta_t.nanosec / 1e9
        return delta_t
        

    def publish_control_cmd(self, steering_angle, linear_speed=None, rear_wheel_angular_speed=None):
        cmd_vel = Twist()
        if linear_speed is not None:
            cmd_vel.linear.x = linear_speed
        else:
            cmd_vel.linear.x = rear_wheel_angular_speed * self.wheel_radius * np.cos(steering_angle) 
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = cmd_vel.linear.x * np.tan(steering_angle) / self.WB
        self.control_command_publisher(cmd_vel)


        


def main(args=None):
    rclpy.init(args=args)
    path_tracking_sub = PathTracking_Subscriber()
    rclpy.spin(path_tracking_sub)
    path_tracking_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
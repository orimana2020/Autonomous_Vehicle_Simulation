#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import GetPath
from AStar_python_interface import A_Star, CSpace
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from PurePursuit_python_interface import PurePersuit_Controller
from utils import euler_from_quaternion, Trajectory, State
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.parameter import Parameter

from nav_msgs.msg import Path


class PathTracking(Node):
    def __init__(self):
        super().__init__('Path_Tracking')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('show_path', False )
        show_path_param = self.get_parameter('show_path').get_parameter_value().bool_value
        # new_custom_show_path_param = rclpy.parameter.Parameter(
        #     'show_path',
        #     rclpy.Parameter.Type.BOOL,
        #     False
        # )
        # self.set_parameters([new_custom_show_path_param])
                            
        
        #TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(timer_period_sec=0.05, callback=self.tf_timer)

        #  hyper-parameters
        k = 0.1  # look forward gain
        Lfc = 1.2  # [m] look-ahead distance
        Kp = 1.0  # speed proportional gain
        self.TargetSpeed = 3.0  # [m/s]
        self.MAX_STEER = np.deg2rad(40.0)  # maximum steering angle [rad]
        MAX_DSTEER = np.deg2rad(150.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = 2.0 # maximum speed [m/s]
        self.MIN_SPEED = 1.0  # minimum speed [m/s]
        MAX_ACCEL = 1.0  # maximum accel [m/ss]
        self.wheel_radius = 0.056
        self.WB = 0.335
        self.max_radial_velocity_rear_wheel = self.MAX_SPEED / self.wheel_radius
        self.pure_persuit_frequency = 2

        self.path = np.load('path_meter.npy')
        
        if show_path_param:
            self.path_pulisher = self.create_publisher(Path, '/path', 1)
            self.path_publisher_timer = self.create_timer(2.0, self.path_callback)

        self.trajectory = Trajectory(dl=0.5, path =self.path, TARGET_SPEED=self.TargetSpeed)
        self.state = State(WB=self.WB, x=self.trajectory.cx[0], y=self.trajectory.cy[0], yaw=self.trajectory.cyaw[0], v=0.0)
        self.lastIndex = len(self.trajectory.cx) - 1
        self.pp = PurePersuit_Controller(self.trajectory.cx, self.trajectory.cy, k, Lfc, Kp, self.WB, MAX_ACCEL, self.MAX_SPEED, self.MIN_SPEED, self.MAX_STEER, MAX_DSTEER)
        self.target_ind, _ = self.pp.search_target_index(self.state)
        self.control_command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        # self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.amcl_pose_callback,10)
        # self.amcl_sub  
    
    def path_callback(self):
        path = Path()
        path.header.frame_id = 'map'
        poses = []
        for i,j in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i
            pose.pose.position.y = j
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path.poses = poses
        self.path_pulisher.publish(path)
            


    
    def tf_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame='map',source_frame='base_link', time=rclpy.time.Time())                
            self.pp_callback(transform)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform ')
            return
    
    def pp_callback(self, pose: TransformStamped):
        self.update_pose(pose)
        print(f'x = {self.state.x:.2f}, y = {self.state.y:.2f}, yaw = {self.state.yaw:.2f} rear_x={self.state.rear_x:.2f} rear_y={self.state.rear_y:.2f}')
    
        delta_t = self.update_time()
        if self.target_ind < self.lastIndex :#and (delta_t > 1 / self.pure_persuit_frequency):
            # self.state.v = self.pp.proportional_control_acceleration(self.TargetSpeed)
            delta, self.target_ind = self.pp.pure_pursuit_steer_control(self.state, self.trajectory, self.target_ind, delta_t)
            self.state.predelta = delta
            # self.publish_control_cmd(steering_angle=delta, linear_speed = self.state.v)
            linear_velocity = self.get_linear_velocity(steering_angle=delta)
            self.publish_control_cmd(steering_angle=delta, rear_wheel_angular_speed = linear_velocity / self.wheel_radius)
            self.prev_time = self.get_clock().now()
    
    def update_pose(self, pose: TransformStamped):
        x = self.state.x = pose.transform.translation.x
        y = self.state.y = pose.transform.translation.y
        _, _, yaw = euler_from_quaternion(quaternion= pose.transform.rotation)
        self.state.update(x, y, yaw)
    
    def get_linear_velocity(self, steering_angle):
        return self.MIN_SPEED + (self.MAX_SPEED -self.MIN_SPEED) * (self.MAX_STEER - abs(steering_angle)) / self.MAX_STEER
  

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
        self.control_command_publisher.publish(cmd_vel)




def main(args=None):
    rclpy.init(args=args)
    path_tracking_sub = PathTracking()
    rclpy.spin(path_tracking_sub)
    path_tracking_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
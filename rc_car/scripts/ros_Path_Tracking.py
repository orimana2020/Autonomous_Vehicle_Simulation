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
import py_car_consts

class PathTracking(Node):
    def __init__(self):
        super().__init__('Path_Tracking')
        # self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self.declare_parameter('show_path', False )
        self.declare_parameter('show_marker', False )
        self.declare_parameter('path_name', 'default_path')
        path_name = self.get_parameter('path_name').get_parameter_value().string_value
        show_path_param = self.get_parameter('show_path').get_parameter_value().bool_value
        self.show_marker_param = self.get_parameter('show_marker').get_parameter_value().bool_value

        #TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(timer_period_sec=0.05, callback=self.tf_timer)

        #  hyper-parameters
        k = 0.1  # look forward gain
        Lfc = 0.7  # [m] look-ahead distance
        Kp = 1.0  # speed proportional gain
        self.TargetSpeed = 0.9  # [m/s]
        self.MAX_STEER = py_car_consts.max_steering_angle_rad  # maximum steering angle [rad]
        MAX_DSTEER = py_car_consts.max_dt_steering_angle  # maximum steering speed [rad/s]
        self.MAX_SPEED = py_car_consts.max_linear_velocity # maximum speed [m/s]
        self.MIN_SPEED = py_car_consts.min_linear_velocity  # minimum speed [m/s]
        MAX_ACCEL = 1.0  # maximum accel [m/ss]
        self.wheel_radius = py_car_consts.wheel_radius
        self.WB = py_car_consts.wheelbase
        self.path = np.load(path_name+'.npy')

        if show_path_param:
            self.path_pulisher = self.create_publisher(Path, '/path', 1)
            self.path_publisher_timer = self.create_timer(2.0, self.path_callback)
            self.marker_pulisher = self.create_publisher(Marker, '/marker', 1)

        self.init_marker()
        self.trajectory = Trajectory(dl=0.5, path =self.path, TARGET_SPEED=self.TargetSpeed)
        self.state = State(WB=self.WB, x=self.trajectory.cx[0], y=self.trajectory.cy[0], yaw=self.trajectory.cyaw[0], v=0.0)
        self.lastIndex = len(self.trajectory.cx) - 1
        self.pp = PurePersuit_Controller(self.trajectory.cx, self.trajectory.cy, k, Lfc, Kp, self.WB, MAX_ACCEL, self.MAX_SPEED, self.MIN_SPEED, self.MAX_STEER, MAX_DSTEER)
        self.target_ind, _ = self.pp.search_target_index(self.state)
        self.control_command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        
        
    
    def path_callback(self):
        path = Path()
        path.header.frame_id = 'map'
        poses = []
        for coords in range(len(self.path)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.path[coords][0]
            pose.pose.position.y = self.path[coords][1]
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
        # print(f'x = {self.state.x:.2f}, y = {self.state.y:.2f}, yaw = {self.state.yaw:.2f} rear_x={self.state.rear_x:.2f} rear_y={self.state.rear_y:.2f}')
        delta_t = self.update_time()
        if self.target_ind < self.lastIndex :
            # self.state.v = self.pp.proportional_control_acceleration(self.TargetSpeed)
            delta, self.target_ind, self.tx, self.ty = self.pp.pure_pursuit_steer_control(self.state, self.trajectory, self.target_ind, delta_t)
            self.state.predelta = delta
            linear_velocity = self.get_linear_velocity(steering_angle=delta)
            # print(f'streering_angle : {np.rad2deg(delta)}, rear_wheel_speed: {linear_velocity / self.wheel_radius}')
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


    def publish_control_cmd(self, steering_angle, rear_wheel_angular_speed):
        cmd_vel = Twist()
        cmd_vel.linear.x = rear_wheel_angular_speed * self.wheel_radius * np.cos(steering_angle) 
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = cmd_vel.linear.x * np.tan(steering_angle) / self.WB
        self.control_command_publisher.publish(cmd_vel)
        if self.show_marker_param:
            self.publish_marker()

    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = 0 
        marker.pose.position.z = 0.2
        marker.scale.x, marker.scale.y, marker.scale.z = 0.2,0.2,0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker = marker
        

    def publish_marker(self):
        self.marker.pose.position.x = self.tx
        self.marker.pose.position.y = self.ty
        self.marker_pulisher.publish(self.marker)

        

def main(args=None):
    rclpy.init(args=args)
    path_tracking_sub = PathTracking()
    rclpy.spin(path_tracking_sub)
    path_tracking_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np


class InverseTwist(Node):
    '''
    this node takes as input steering angle and radial velocity of the rear wheel
    and convert it to twist_msg
    '''
    def __init__(self):
        super().__init__('inverse_twist')
        # self.declare_parameters(namespace='',
        #                         parameters=[
        #                                     ('max_linear_velocity', 1.0),
        #                                     ('max_angular_velocity', 1.0),
        #                                     ])
        # self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        # self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        
        
        self.create_subscription(Joy, '/joy',self.feedback_callback_joy,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_joy_inverse', 10)

        self.wheel_radius = 0.056
        self.wheelbase = 0.3429
        # if max_steering_angle changed the value in ROS_ARDUINO_BRIDGE must be changed too!
        self.max_steering_angle = 0.6 #  = 35[deg] * np.pi / 180
        max_linear_velocity = 2
        self.max_radial_velocity_rear_wheel = max_linear_velocity / self.wheel_radius 

    def feedback_callback_joy(self, joy_msg):
        enable = joy_msg.axes[2] # enable buttum
        if enable < 0:
            rear_wheel_w, steering_angle  = self.mapping(joy_msg.axes[1], joy_msg.axes[3])
            cmd_vel = Twist()
            cmd_vel.linear.x = rear_wheel_w * self.wheel_radius * np.cos(steering_angle) 
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = cmd_vel.linear.x * np.tan(steering_angle) / self.wheelbase
            self.publisher.publish(cmd_vel)
    
    def mapping(self, linear_velocity, steering_angle):
        return linear_velocity * self.max_radial_velocity_rear_wheel, steering_angle * self.max_steering_angle




def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = InverseTwist()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

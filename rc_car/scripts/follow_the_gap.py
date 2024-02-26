#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from builtin_interfaces.msg import Duration




class FollowTheGap(Node):

    def __init__(self):
        super().__init__('follow_the_gap')
        self.create_subscription(LaserScan, '/scan', self.feedback_callback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.wheel_radius = 0.056 # [m]
        self.wheelbase = 0.3429 # [m]
        self.car_width = 0.2 # [m]
        self.eliminate_facetor = 1.5
        # if max_steering_angle changed the value in ROS_ARDUINO_BRIDGE must be changed too!
        self.max_steering_angle = 0.6 #  = 35[deg] * np.pi / 180
        max_linear_velocity = 2
        self.max_radial_velocity_rear_wheel = max_linear_velocity / self.wheel_radius 
        self.laser_angular_range = 270
        self.offset = int((360 - self.laser_angular_range) /2) 
        self.threshsold = 2.5 # [m]
        

    def feedback_callback(self, msg:LaserScan):
        ranges = []
        for i in range(self.laser_angular_range):
            if msg.ranges[i+self.offset] > self.threshsold: 
                ranges.append(1)
            else:
                ranges.append(0)
        i_start_max_count = 0
        i_start_count = 0
        counts = 0
        max_counts = 0
        for i in range(self.laser_angular_range):
            if ranges[i] == 1:
                if counts ==0:
                    i_start_count = i
                counts += 1
            else:
                if max_counts < counts:
                    max_counts = counts
                    i_start_max_count = i_start_count 
                counts = 0
        direction = (-1* (self.laser_angular_range/2 - (i_start_max_count + max_counts / 2 ))) * np.pi / 180
        self.publish_steering_cmd(steering_angle=direction)
        
   
    def publish_steering_cmd(self, steering_angle):
        rear_wheel_w = self.max_radial_velocity_rear_wheel
        steering_angle  = steering_angle
        cmd_vel = Twist()
        cmd_vel.linear.x = rear_wheel_w * self.wheel_radius * np.cos(steering_angle) 
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = cmd_vel.linear.x * np.tan(steering_angle) / self.wheelbase
        self.publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    follow_the_gap = FollowTheGap()
    rclpy.spin(follow_the_gap)
    follow_the_gap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

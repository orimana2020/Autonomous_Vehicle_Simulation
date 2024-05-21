#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from builtin_interfaces.msg import Duration

class PID(object):
    def __init__(self, current_time:Duration):
        self.error = 0
        self.Prev_error = 0
        self.Integral = 0
        self.kp = 0.5
        self.kd = 0
        self.ki = 0
        self.prev_time = current_time
    
    def pid_step(self, ref, current, time:Duration):
        # self.error = current - ref 
        self.error = ref - current
        delta_t = (time - self.prev_time).to_msg()
        delta_t = delta_t.sec + delta_t.nanosec / 1e9
        self.prev_time = time
        dedt = (self.error - self.Prev_error) / delta_t
        steering_command = self.error * self.kp +  self.kd * dedt  + self.Integral * self.ki
        self.Prev_error = self.error
        self.Integral += self.error * delta_t
        return steering_command



class WALL_FOLLOWING(Node):
    '''
    this node takes as input steering angle and radial velocity of the rear wheel
    and convert it to twist_msg
    '''
    def __init__(self):
        super().__init__('wall_following')
        self.theta = 70
        self.theta_rad = self.theta * np.pi / 180
        self.L = 1.5
    
        self.create_subscription(LaserScan, '/scan', self.feedback_callback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.wheel_radius = 0.056
        self.wheelbase = 0.3429
        # if max_steering_angle changed the value in ROS_ARDUINO_BRIDGE must be changed too!
        self.max_steering_angle = 0.6 #  = 35[deg] * np.pi / 180
        max_linear_velocity = 2
        self.max_radial_velocity_rear_wheel = max_linear_velocity / self.wheel_radius 
        self.pid = PID(current_time=self.get_clock().now())
        

    def feedback_callback(self, msg:LaserScan):
        right_beam = msg.ranges[90]
        
        offset_beam = msg.ranges[90+self.theta] 
        alpha = np.arctan((offset_beam*np.cos(self.theta_rad) - right_beam) / (offset_beam*np.sin(self.theta_rad)))
        dist2wall = right_beam * np.cos(alpha)
        
        predicted_dist2wall = dist2wall + self.L * np.sin(alpha)
        steering_command = self.pid.pid_step(ref=1.0 , current=predicted_dist2wall, time = self.get_clock().now())
        print(f'right_beam {right_beam}, offset_beam {offset_beam}, predicted_dist2wall {predicted_dist2wall}, steering angle {steering_command}')
        self.publish_steering_cmd(steering_angle=steering_command)
        
       

   
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
    wall_following = WALL_FOLLOWING()
    rclpy.spin(wall_following)
    wall_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

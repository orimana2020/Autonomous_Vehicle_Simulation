#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('max_linear_velocity', 1.0),
                                    ('max_angular_velocity', 1.0),
                                ])
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        
        self.create_subscription(Joy, '/joy',self.feedback_callback_joy,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_joy', 10)


    def feedback_callback_joy(self, joy_msg):
        enable = joy_msg.axes[2] # enable buttum
        if enable < 0:
            cmd_vel = Twist()
            cmd_vel.linear.x = joy_msg.axes[1] * self.max_linear_velocity
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = joy_msg.axes[3]
            self.publisher.publish(cmd_vel)



def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = TeleopPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

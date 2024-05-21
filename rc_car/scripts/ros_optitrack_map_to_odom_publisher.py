#!/usr/bin/env python3
import math
from typing import NamedTuple
import numpy as np
import rclpy
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TwistWithCovariance, Vector3)
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.parameter import Parameter
from control_msgs.msg import SteeringControllerStatus
import py_car_consts
"""
subscribe to /diff_cont/controller_state [control_msgs/msg/SteeringControllerStatus], 
calculate odometry, and publish to /odom and /tf
"""


class MapOdomPublisher(Node):
    """Listens to odometry information and publishes Odometry message at regular rate"""

    def __init__(self):
        super().__init__('map_to_odom_publisher')

        # Publishers
        queue_size = 10
        # self.publisher_odom = self.create_publisher(Odometry, 'odom', queue_size)
        self.publisher_tf = self.create_publisher(TFMessage, 'tf', queue_size)


        # Subscribers
        self.create_subscription(
            SteeringControllerStatus,
            '/diff_cont/controller_state',
            self.feedback_callback,
            10
        )

    


    def output(self):
       

        tf_msg = TFMessage()
        tf_msg.transforms.append(TransformStamped())
        tf_msg.transforms[0].header.stamp = self.get_clock().now().to_msg()
        tf_msg.transforms[0].header.frame_id = "map"
        tf_msg.transforms[0].child_frame_id = "odom"
        tf_msg.transforms[0].transform.translation.x = 0.0
        tf_msg.transforms[0].transform.translation.y = 0.0
        tf_msg.transforms[0].transform.translation.z = 0.0
        tf_msg.transforms[0].transform.rotation.x = 0.0
        tf_msg.transforms[0].transform.rotation.y = 0.0
        tf_msg.transforms[0].transform.rotation.z = 0.0
        tf_msg.transforms[0].transform.rotation.w = 1.0
        
        # odom_msg =  Odometry(
        #     header=Header(
        #         stamp=state.time.to_msg(),
        #         frame_id='odom'
        #     ),
        #     child_frame_id="base_link",
        #     pose=PoseWithCovariance(
        #         pose=Pose(
        #             position=Point(
        #                 x=state.position[0],
        #                 y=state.position[1],
        #                 z=state.position[2]
        #             ),
        #             orientation=Quaternion(
        #                 x=quaternion[0],
        #                 y=quaternion[1],
        #                 z=quaternion[2],
        #                 w=quaternion[3]
        #             )
        #         )
        #     ),
        #     twist=TwistWithCovariance(
        #         twist=Twist(
        #             linear=Vector3(
        #                 x=linear_velocity[0],
        #                 y=linear_velocity[1],
        #                 z=linear_velocity[2]
        #             ),
        #             angular=Vector3(
        #                 z=angular_speed
        #             )
        #         )
        #     )
        # )

        return tf_msg #, odom_msg

    def feedback_callback(self, msg: SteeringControllerStatus):
        """Update state and publish to Odometry"""
        # self.state = self.state_update(self.state, msg)
        # self.get_logger().debug(f'state update: {self.state}')
        # output = self.output(self.state)
        tf_msg = self.output()
        # self.get_logger().debug(f'odometry: {output}')
        self.publisher_tf.publish(tf_msg)
        



def main(args=None):
    rclpy.init(args=args)
    odom_publisher = MapOdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

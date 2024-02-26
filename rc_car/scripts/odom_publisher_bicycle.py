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
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from rclpy.parameter import Parameter
from control_msgs.msg import SteeringControllerStatus
"""
subscribe to /diff_cont/controller_state [control_msgs/msg/SteeringControllerStatus], 
calculate odometry, and publish to /odom and /tf
"""

class AckermannState(NamedTuple):
    position: np.array
    orientation: Rotation
    track_wheel_speed: float
    steering_angle: float
    time: rclpy.time.Time


class OdomPublisher(Node):
    """Listens to odometry information and publishes Odometry message at regular rate"""

    def __init__(self):
        super().__init__('odom_publisher')
        # self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.wheelbase_length = 0.3429
        self.wheel_radius = 0.056
        self.center_of_mass_offset = 0.0
        self.damping_factor = 1.0

        # Publishers
        queue_size = 10
        # msg_type = 'tf2_msgs'
        # if msg_type == 'nav_msgs':
        self.publisher_odom = self.create_publisher(Odometry, 'odom', queue_size)
        # elif msg_type == 'tf2_msgs':
        self.publisher_tf = self.create_publisher(TFMessage, 'tf', queue_size)


        # Subscribers
        self.create_subscription(
            SteeringControllerStatus,
            '/diff_cont/controller_state',
            self.feedback_callback,
            10
        )

        self.state = AckermannState(
            position=np.array([0,0,0]),
            orientation=Rotation([0, 0, 0, 1]), # identity
            track_wheel_speed=0.0,
            steering_angle=0.0,
            time=self.get_clock().now()
        )

    def state_update(self, state: AckermannState, feedback: SteeringControllerStatus) -> AckermannState:
        """Calculate the next state based off the current state"""
      
        average_wheel_speed = state.track_wheel_speed
        linear_speed = average_wheel_speed * self.wheel_radius
        turn_radius = self.turn_radius(state.steering_angle)
        angular_speed = linear_speed / turn_radius # This is zero if turn_radius is infinite

        feedback_time = rclpy.time.Time.from_msg(feedback.header.stamp)
        time_delta = (feedback_time - state.time).nanoseconds * 1e-9

        heading_delta = angular_speed * time_delta # This is zero if angular_speed is zero

        orientation_delta = Rotation.from_euler('xyz', [0, 0, heading_delta])
        if math.isfinite(turn_radius):
            lateral_delta = turn_radius * (1 - math.cos(heading_delta))
            forward_delta = turn_radius * math.sin(heading_delta)
            relative_delta = np.array([forward_delta, lateral_delta, 0])
            position_delta = state.orientation.apply(relative_delta)
        else:
            position_delta = time_delta * self.linear_velocity(state.orientation, linear_speed)

        return AckermannState(
            position= state.position + self.damping_factor * position_delta,
            orientation = orientation_delta * state.orientation,
            track_wheel_speed = feedback.traction_wheels_velocity[0],
            steering_angle = feedback.steer_positions[0],
            time = feedback_time
        )

    def output(self, state: AckermannState) -> Odometry:
        """Build Odometry message from state"""
        quaternion = state.orientation.as_quat()
        linear_speed = self.wheel_radius * state.track_wheel_speed
        linear_velocity = self.linear_velocity(state.orientation, linear_speed)
        angular_speed = linear_speed / self.turn_radius(state.steering_angle)

        tf_msg = TFMessage()
        tf_msg.transforms.append(TransformStamped())
        tf_msg.transforms[0].header.stamp = state.time.to_msg()
        tf_msg.transforms[0].header.frame_id = "odom"
        tf_msg.transforms[0].child_frame_id = "base_link"
        tf_msg.transforms[0].transform.translation.x = state.position[0]
        tf_msg.transforms[0].transform.translation.y = state.position[1]
        tf_msg.transforms[0].transform.translation.z = state.position[2]
        tf_msg.transforms[0].transform.rotation.x = quaternion[0]
        tf_msg.transforms[0].transform.rotation.y = quaternion[1]
        tf_msg.transforms[0].transform.rotation.z = quaternion[2]
        tf_msg.transforms[0].transform.rotation.w = quaternion[3]
        
        odom_msg =  Odometry(
            header=Header(
                stamp=state.time.to_msg(),
                frame_id='odom'
            ),
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=state.position[0],
                        y=state.position[1],
                        z=state.position[2]
                    ),
                    orientation=Quaternion(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2],
                        w=quaternion[3]
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=linear_velocity[0],
                        y=linear_velocity[1],
                        z=linear_velocity[2]
                    ),
                    angular=Vector3(
                        z=angular_speed
                    )
                )
            )
        )

        return tf_msg, odom_msg

    def feedback_callback(self, msg: SteeringControllerStatus):
        """Update state and publish to Odometry"""
        self.state = self.state_update(self.state, msg)
        self.get_logger().debug(f'state update: {self.state}')
        # output = self.output(self.state)
        tf_msg, odom_msg = self.output(self.state)
        # self.get_logger().debug(f'odometry: {output}')
        self.publisher_tf.publish(tf_msg)
        self.publisher_odom.publish(odom_msg)
        # self.publisher.publish(output)

    def turn_radius(self, steering_angle: float) -> float:
        """
        Calculate the radius of the circle tracked by a turning vehicle
        For left turns, the radius is positive. For right turns, the radius is negative.
        """
        # If the steering angle is 0, then cot(0) is undefined
        if steering_angle == 0:
            return math.inf
        else:
            radius = math.sqrt(self.center_of_mass_offset**2 + self.wheelbase_length**2 * 1 / math.tan(steering_angle)**2)
            return math.copysign(radius, steering_angle)

    def linear_velocity(self, orientation: Rotation, speed: float) -> np.array:
        return orientation.apply([speed, 0, 0]) # rotate from +x direction

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

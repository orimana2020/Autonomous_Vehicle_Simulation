#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
from builtin_interfaces.msg import Duration
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class Path_Planning(Node):
    # '''
    # this node takes as input steering angle and radial velocity of the rear wheel
    # and convert it to twist_msg
    # '''
    def __init__(self):
        super().__init__('path_planning')
        self.theta = 70
        self.theta_rad = self.theta * np.pi / 180
        self.L = 1.5
    
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # request srv to load the once again becasuse it published only once
        self.cli = self.create_client(LoadMap, '/map_server/load_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.req.map_url = './src/rc_car/maps/maze_1/maze1_map.yaml'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)


        #TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.tf_timer)

        # ----------
        
        self.wheel_radius = 0.056
        self.wheelbase = 0.3429
        # if max_steering_angle changed the value in ROS_ARDUINO_BRIDGE must be changed too!
        self.max_steering_angle = 0.6 #  = 35[deg] * np.pi / 180
        max_linear_velocity = 2
        self.max_radial_velocity_rear_wheel = max_linear_velocity / self.wheel_radius 

    def tf_timer(self):
        from_frame_rel = 'map'
        to_frame_rel = 'base_link'
        self.transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.transform = transform
            print(transform)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

                  
            

    def map_callback(self, map:OccupancyGrid):
        self.map_msg = map
        self.resolution = self.map_msg.info.resolution 
        self.origin_meter = self.map_msg.info.origin #geometry_msgs.msg.Pose, the lower-left pixel in the map
        self.origin_x_meter = self.origin_meter.position.x
        self.origin_y_meter = self.origin_meter.position.y
        self.origin_x_pixel = self.origin_meter.position.x / self.resolution
        self.origin_y_pixel = self.origin_meter.position.y / self.resolution
        self.orientation = self.origin_meter.orientation
        self.map_width_x = self.map_msg.info.width
        self.map_height_y = self.map_msg.info.height
        self.map_data_row = self.map_msg.data
        self.map = [] 
        for i in range(0, self.map_width_x * self.map_height_y, self.map_width_x):
            self.map.append(self.map_data_row[i:i+self.map_width_x])
        self.map = np.array(self.map)
        print(self.map)
    

    def meter_to_pixel(self, x_meter, y_meter):
        x_pixel = max(int((x_meter - self.origin_x_meter) / self.resolution), 0)
        y_pixel = max(int((y_meter - self.origin_y_meter) / self.resolution), 0)
        return x_pixel, y_pixel
    

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
    wall_following = Path_Planning()
    rclpy.spin(wall_following)
    wall_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

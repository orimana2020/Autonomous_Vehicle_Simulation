#!/usr/bin/env python3

import sys
from interfaces.srv import GetPath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from AStar_python_interface import CSpace
from nav_msgs.srv import GetMap
import numpy as np


class PathPlanning_Client(Node):
    def __init__(self):
        super().__init__('calc_path_client')
        self.cli = self.create_client(GetPath, '/calc_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPath.Request()
        # self.get_map()

    def send_request(self, start, goal):
        self.req.start = start
        self.req.goal = goal
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    # def get_map(self):
    #     self.map_client = self.create_client(GetMap, '/map_server/map')
    #     while not self.map_client.wait_for_service(timeout_sec=3.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     self.map_req = GetMap.Request()
    #     self.map_future = self.map_client.call_async(self.map_req)
    #     rclpy.spin_until_future_complete(self, self.map_future)
    #     map_response = self.map_future.result()
    #     print(f'map response :{map_response}')
    #     self.map_origin = map_response.map.info.origin
    #     self.map_resolution = map_response.map.info.resolution
    #     map_data = np.array(map_response.map.data)
    #     self.map_width = map_response.map.info.width
    #     self.map_height = map_response.map.info.height
    #     self.map_origin_x = map_response.map.info.origin.position.x
    #     self.map_origin_y = map_response.map.info.origin.position.y
    #     self.map_data = map_data.reshape((self.map_height, self.map_width))


def main():
    rclpy.init()
    path_planning_client = PathPlanning_Client()
    start = Point()
    start.x = 0.0
    start.y = 0.0
    goal = Point()
    goal.x = 6.22
    goal.y = -4.5
    
    response = path_planning_client.send_request(start, goal)
    print(response.path)
    path_planning_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
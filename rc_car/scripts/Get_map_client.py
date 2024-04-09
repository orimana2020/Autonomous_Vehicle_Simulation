#!/usr/bin/env python3

import sys
from nav_msgs.srv import GetMap
import rclpy
from rclpy.node import Node
import numpy as np

class GetMap_ClientAsync(Node):
    """
    send request to service to get the loaded map 
    """
    def __init__(self):
        super().__init__('get_map')
        self.cli = self.create_client(GetMap, '/map_server/map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    get_map_client = GetMap_ClientAsync()
    response = get_map_client.send_request()
    map_origin = response.map.info.origin.position
    print(f'origin {map_origin}')
    map_resolution = response.map.info.resolution
    print(f'resolution {map_resolution}')
    map_width = response.map.info.width
    print(f'width {map_width}')
    map_height = response.map.info.height
    print(f'heigth {map_height}')
    map_origin_x = response.map.info.origin.position.x
    print(f'origin_x {map_origin_x}')
    map_origin_y = response.map.info.origin.position.y
    print(f'origin_y {map_origin_y}')
    map_data = np.array(response.map.data)
    print(f'data {map_data}')
    map_data = map_data.reshape((map_height, map_width))
    print(f'data {map_data}')
    np.save('maze_test', map_data)
    get_map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import sys
from nav_msgs.srv import GetMap
import rclpy
from rclpy.node import Node

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
    origin = response.info.origin
    resolution = response.info.resolution
    map_data = response.info.data
    width = response.info.width
    height = response.info.height
    print(map_data)
    get_map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
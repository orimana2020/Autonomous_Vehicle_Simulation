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
        self.declare_parameter('map_name', 'default_map')
        self.map_name = str(self.get_parameter('map_name').get_parameter_value().string_value)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response =  self.future.result()
        map_resolution = float(response.map.info.resolution)
        print(f'resolution {map_resolution}')
        map_width = int(response.map.info.width)
        print(f'width {map_width}')
        map_height = int(response.map.info.height)
        print(f'heigth {map_height}')
        map_origin_x = float(response.map.info.origin.position.x)
        print(f'origin_x {map_origin_x}')
        map_origin_y = float(response.map.info.origin.position.y)
        print(f'origin_y {map_origin_y}')
        map_origin_z = float(response.map.info.origin.position.z)
        print(f'origin_z {map_origin_z}')
        map_data = np.array(response.map.data)
        # print(f'data {map_data}')
        map_data = map_data.reshape((map_height, map_width))
        print(f'data {map_data}')
        map_dict = { 'map_resolution': map_resolution, 'map_width':map_width,'map_height':map_height,
                   'map_origin_x':map_origin_x, 'map_origin_y':map_origin_y,'map_origin_z':map_origin_z ,'map_data':map_data }
        np.save(self.map_name, map_dict)
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)
    get_map_client = GetMap_ClientAsync()
    response = get_map_client.send_request()
    get_map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
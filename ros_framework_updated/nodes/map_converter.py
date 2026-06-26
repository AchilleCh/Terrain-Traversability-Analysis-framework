#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import numpy as np


class map_converter_node(Node):
    def __init__(self):
        super().__init__("map_converter_node")
        self.map_sub = self.create_subscription(Costmap, "/topic_final_cmap", self.converter_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/topic_final_grid", 10)


    def converter_callback(self, map:Costmap):
        grid_msg = OccupancyGrid()
        grid_msg.header = map.header
        grid_msg.header.frame_id = map.header.frame_id
        grid_msg.info.map_load_time = map.header.stamp
        grid_msg.info.resolution = map.metadata.resolution
        grid_msg.info.width = map.metadata.size_x
        grid_msg.info.height = map.metadata.size_y
        grid_msg.info.origin.position.x = map.metadata.origin.position.x
        grid_msg.info.origin.position.y = map.metadata.origin.position.y
        grid_msg.info.origin.position.z = map.metadata.origin.position.z
        grid_msg.info.origin.orientation.w = map.metadata.origin.orientation.w

        map_data = np.frombuffer(map.data, dtype=np.uint8)
        grid_msg.data = map_data.astype(np.int8).tolist()

        self.grid_pub.publish(grid_msg)
        


  



def main(args = None):
    rclpy.init(args = args)
    node = map_converter_node()
    rclpy.spin(node = node)
    rclpy.shutdown()
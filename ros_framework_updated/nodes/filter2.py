#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import Image
import numpy as np

# Filter to synchronize messages
class filter2(Node):
    def __init__(self):
        super().__init__("filter_node2")
        self.get_logger().info("filter_node started")
        self.geom_cmap_sub = Subscriber(self,Costmap, "/topic_G_cmap")
        self.app_cmap_sub = Subscriber(self,Costmap, "/topic_A_cmap")
        self.grid_G_sub = Subscriber(self,Image, "/topic_G_grid")
        

        self.geom_cmap_pub = self.create_publisher(Costmap, "/topic_G_cmap_OUT", 10)
        self.app_cmap_pub = self.create_publisher(Costmap, "/topic_A_cmap_OUT", 10)
        self.grid_G_pub = self.create_publisher(Image, "/topic_G_grid_OUT", 10)

        
        self.sync = ApproximateTimeSynchronizer([self.geom_cmap_sub, self.app_cmap_sub, self.grid_G_sub],10,2.0) #create the message synchronizer filter
        self.sync.registerCallback(self.syncCallback)
    

    def syncCallback(self, geom_cmap:Costmap, app_cmap:Costmap, grid_corresp:Image):
        # publish synchronized data
        self.geom_cmap_pub.publish(geom_cmap)
        self.app_cmap_pub.publish(app_cmap)
        self.grid_G_pub.publish(grid_corresp)
        self.get_logger().warning("All synchronized data published")

    
        



def main(args = None):
    rclpy.init(args = args)
    node = filter2()
    rclpy.spin(node = node)
    rclpy.shutdown()
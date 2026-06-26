#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image
import numpy as np
import os
import cv2

class publisher_node(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.get_logger().info("Beginning of data publishing")
        self.rgb_path = "/home/user/Thesis/Dataset/synthetic_set/images/test/im_test_999.png"
        self.depth_path = "/home/user/Thesis/Dataset/synthetic_set/depths/test/depth_test_999.exr"
        
        os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

        self.rgb_img = cv2.imread(self.rgb_path)
        self.rgb_img = self.rgb_img[:,:,::-1]
        self.depth_img = cv2.imread(self.depth_path,cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        #self.get_logger().info(f"max depth: {np.max(self.depth_img)}")

        self.depth_pub = self.create_publisher(Image, '/topic_depth_IN',10)
        self.img_pub = self.create_publisher(Image, '/topic_img_IN',10)
        self.timer = self.create_timer(0.1, self.publish_callback)    
        

    def publish_callback(self):
        #initialize the messages

        #self.get_logger().info(f"max depth published {np.max(self.depth_img)}")
        img_msg = Image() 
        #https://docs.ros.org/en/ros2_packages/rolling/api/depth_image_proc/ ---> uses Image messages
        depth_msg = Image()

        img_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        
        img_msg.width = self.rgb_img.shape[0]
        img_msg.height = self.rgb_img.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.step = img_msg.width * 3
        data = np.copy(self.rgb_img)
        data = data.astype(dtype=np.uint8)
        flattened_data = np.ndarray.flatten(data,'C')
        img_msg.data = flattened_data.tobytes()

        depth_msg.width = self.depth_img.shape[0]
        depth_msg.height = self.depth_img.shape[1]
        depth_msg.encoding = "mono16"
        depth_msg.step = img_msg.width * 2 #no color channels just the depth channel
        depth_data = np.copy(self.depth_img[:,:,0])
        depth_data = depth_data * 1000
        depth_data = depth_data.astype(dtype=np.uint16)
        flattened_data_depth = np.ndarray.flatten(depth_data,'C')
        depth_msg.data = flattened_data_depth.tobytes()
        

        

        self.img_pub.publish(msg = img_msg)
        self.depth_pub.publish(msg = depth_msg)

        
        
        
        




def main(args = None):
    rclpy.init(args=args)
    os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
    node = publisher_node()
    rclpy.spin(node=node)
    rclpy.shutdown()
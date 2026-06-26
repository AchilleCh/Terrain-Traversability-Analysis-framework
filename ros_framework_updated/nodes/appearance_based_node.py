#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped # Message for the transformation matrix
import rclpy.time
from sensor_msgs.msg import Image
from nav2_msgs.msg import Costmap #Costmap message imported from nav2 package
from tf2_ros.transform_listener import TransformListener #Needed to retrieve transformation matrices
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import torch
import albumentations as A
from albumentations.pytorch import ToTensorV2
import sys
sys.path.append('/home/user/ros2_ws/src/ros_framework/ros_framework/') # Add the path to the folder containing all the needed scripts
from Unet_other_version import UNet 
from utils_app_node import infer_seg_cost_map, load_checkpoint, infer_class_cost_map


class appearance_node(Node):
    def __init__(self):
        super().__init__("appearance_node")
        self.get_logger().info("appearance_node started")
        
        # Definition of the reference frames
        self.wf = 'odom'
        self.sf = 'camera'

        # 3x4 transformation matrix from 2d image plane to sensor frame
        self.sensor_to_2D_matrix = np.array([[541.4,0,0,320],[0,541.4,0,240],[0,0,0,1]]) 

        if torch.cuda.is_available():
            self.DEVICE = "cuda:0"
        else:
            self.DEVICE = "cpu"
        
        self.model_seg = UNet(in_channels=3, out_channels=4, init_features=64).to(self.DEVICE)
        self.model_class = UNet(in_channels=3, out_channels=4, init_features=64).to(self.DEVICE)
        
        self.w_seg = 0.5
        self.w_class = 0.5

        load_checkpoint(self.model_seg, "S")
        load_checkpoint(self.model_class, "C")

        self.get_logger().info("NNs ready")

        # Transformation to apply to the image, change here the resize dimensions
        self.tranformation = A.Compose([A.Resize(height=256,width=256),
                          A.Normalize(mean=[0.0,0.0,0.0],
                                      std=[1.0,1.0,1.0],
                                      max_pixel_value=255.0),
                          ToTensorV2()]
                            )
        # tf listener
        self.tf_buffer = Buffer(cache_time= rclpy.duration.Duration(seconds=10))
        self.tf_subscriber = TransformListener(self.tf_buffer, self)
        
        # RGB listener--> appearance based method employed here
        self.image_subscriber = self.create_subscription(Image,"/topic_img_IN", self.app_callback, 10)
        
        # Cmap publisher
        self.Acmap_publisher = self.create_publisher(Costmap, "/topic_A_cmap", 10)

    # Function to restructure the transformation matrix from the transform stamped message
    def restructure_tf(self, matrix:TransformStamped):

        tf_matrix = np.zeros((4,4))

        qw = matrix.transform.rotation.w
        qx = matrix.transform.rotation.x
        qy = matrix.transform.rotation.y
        qz = matrix.transform.rotation.z

        tf_matrix[0,3] = matrix.transform.translation.x
        tf_matrix[1,3] = matrix.transform.translation.y
        tf_matrix[2,3] = matrix.transform.translation.z
        tf_matrix[3,3] = 1

        tf_matrix[0,0] = qw**2 + qx**2 - qy**2 - qz**2
        tf_matrix[0,1] = 2*(qw*qy - qz*qw)
        tf_matrix[0,2] = 2*(qx*qz + qy*qw)

        tf_matrix[1,0] = 2*(qx*qy + qz*qw)
        tf_matrix[1,1] = qw**2 - qx**2 + qy**2 - qz**2
        tf_matrix[1,2] = 2*(qy*qz - qx*qw)

        tf_matrix[2,0] = 2*(qx*qz - qy*qw)
        tf_matrix[2,1] = 2*(qy*qz + qx*qw)
        tf_matrix[2,2] = qw**2 - qx**2 - qy**2 + qz**2

        tf_matrix[3,0] = 0
        tf_matrix[3,1] = 0
        tf_matrix[3,2] = 0
        
        #self.get_logger().info("Transformation matrix obtained")
        return tf_matrix
    
    # Function to restructure the image
    def restructure_image(self, array, width, height, channels):
        if channels == 3:
            array = np.frombuffer(array, dtype=np.uint8)
            image = array.reshape((width, height, channels))

        elif channels == 1:
            array = np.frombuffer(array, dtype=np.uint16)
            image = array.reshape((width, height))
        
        return image
    


    def point_to_cell(self,x,y,minx,miny,resolution,dimx_map,dimy_map):
        gridx = int((x - minx) / resolution)
        gridy = int((y - miny) / resolution)

        # To ensure the index fall in the correct range
        gridx = min(max(gridx, 0), dimx_map - 1)
        gridy = min(max(gridy, 0), dimy_map - 1)

        if 0 <= gridx < dimx_map and 0 <= gridy < dimy_map:
            return gridx, gridy
        else:
            self.get_logger().error("Point is outside the grid")


    def transform_cmap(self,cmap, pix_to_s, s_to_w):

        tf_matrix = np.matmul(s_to_w, pix_to_s) #transforms from pixel to wf coordinates
        coordinates = np.zeros((cmap.shape[0] * cmap.shape[1],5))
        index = 0
        # creates an array of point coordinates in the wf associated with the relative cost
        for i, row in enumerate(cmap):
            for j, el in enumerate(row):
                pix_coord = np.array([i,j,1])
                coordinates[index, 0:4] = np.matmul(tf_matrix,pix_coord)
                coordinates[index, 4] = el

                index += 1
            
        
        cells_number = 256
        minx = np.min(coordinates[:,0])
        maxx = np.max(coordinates[:,0])
        miny = np.min(coordinates[:,1])
        maxy = np.max(coordinates[:,1])

        map = np.zeros((cells_number,cells_number,2))
        final_map = np.zeros((cells_number,cells_number))

        extent_x_cloud = maxx - minx
        extent_y_cloud = maxy - miny
                

        width_cell =  extent_x_cloud / cells_number
        height_cell = extent_y_cloud / cells_number

        resolution_cell = max(width_cell, height_cell) #to consider squares


        for point in coordinates:
            x = point[0]
            y = point[1]
            cost = point[4]

            gridx, gridy = self.point_to_cell(x , y, minx, miny, resolution_cell, cells_number, cells_number)

            if gridx and gridy:

                map[gridx,gridy,0] += cost
                map[gridx,gridy,1] += 1

        
        for i, row in enumerate(final_map):
            for j, _ in enumerate(row):
                if map[i,j,1] > 0: #to avoid division by zero
                    final_map[i,j] = map[i,j,0]/map[i,j,1]
        


        return final_map


    # Listener callback
    def app_callback(self, image: Image):
        # Image preprocessing
        rgb_image_data = image.data
        rgb_image = self.restructure_image(array = rgb_image_data, width = image.width, height = image.height, channels = 3)
        rgb_image = self.tranformation(image = rgb_image)["image"]
        rgb_image = rgb_image.unsqueeze(0)
        #self.get_logger().info("RGB image recieved")

    
        # Lookup of transformation matrix
        # Retry mechanism to avoid not having transformation available
        try_count = 0
        max_tries = 5
        tf_message = None

        while tf_message is None and try_count < max_tries:

            try:
                tf_message = self.tf_buffer.lookup_transform(self.wf,self.sf,image.header.stamp,
                                                        rclpy.duration.Duration(seconds = 1.0))
                #self.get_logger().info("entered in try")
                try_count += 1

            except TransformException as ex:
                self.get_logger().info(f"Could not transform from {self.sf} to {self.wf}: {ex}")
                rclpy.spin_once(self, timeout_sec=0.1)
                try_count += 1
        
        if tf_message is None:
            self.get_logger().error(f"Failed to get the transformation after {max_tries} tries")
            return 
        

        tf_matrix = self.restructure_tf(tf_message) #restructure of the matrix from the message

        
        cmap_seg, _, _ = infer_seg_cost_map(self.model_seg, rgb_image, self.DEVICE)
        cmap_class, _, _ = infer_class_cost_map(self.model_class, rgb_image, self.DEVICE)


        app_based_cmap = self.w_seg * cmap_seg + self.w_class * cmap_class #combination of cmaps

        # Transform the cmap into wf coordinates
        app_based_cmap = self.transform_cmap(cmap = app_based_cmap, pix_to_s = np.linalg.pinv(self.sensor_to_2D_matrix), s_to_w = tf_matrix)
        app_based_cmap = np.floor_divide(app_based_cmap, (1/255))
        app_based_cmap = app_based_cmap.astype(np.uint8)



        # Create cmap message
        cmap_msg = Costmap()
        # To restructure the cmap since must be sent as a 1d array
        cmap_msg.header.stamp = self.get_clock().now().to_msg()
        cmap_msg.header.frame_id = "odom"
        cmap_msg.metadata.size_x = app_based_cmap.shape[1] #number of columns of the map
        cmap_msg.metadata.size_y = app_based_cmap.shape[0] #number of rows of the map
        flattened_map = np.ndarray.flatten(app_based_cmap,'C')


        cmap_msg.data = flattened_map.tobytes()


        # Publish the appearance based cmap
        self.Acmap_publisher.publish(cmap_msg)



def main(args=None):
    rclpy.init(args = args)
    node = appearance_node()
    rclpy.spin(node = node)
    rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
import open3d
import numpy as np
from math import pi, sqrt
import rclpy.time
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TransformStamped #message for the transformation matrix
#costmap message comes from nav2 package
from nav2_msgs.msg import Costmap
#needed to retrieve transformation matrices
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException


class geometry_node(Node):
    def __init__(self):
        super().__init__("geometry_node")
        self.get_logger().info("geometry_node started")
        
        # Reference frames
        self.wf = 'odom'
        self.sf = 'camera'

        # Camera Matrix Parameters(dimx,dimy,fx,fy,cx,cy)
        self.cx = 320
        self.cy = 240
        self.fx = 541.14
        self.fy = 541.14
        self.camera = open3d.camera.PinholeCameraIntrinsic(1024,1024,self.fx,self.fy,self.cx,self.cy)

        self.preliminary_T = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        
        # tf listener
        self.tf_buffer = Buffer(cache_time= rclpy.duration.Duration(seconds=10))
        self.tf_subscriber = TransformListener(self.tf_buffer, self)

        self.rgb_subscriber = Subscriber(self,Image,"/topic_img_IN")
        self.depth_subscriber = Subscriber(self,Image,"/topic_depth_IN")
        self.synchronyzer = ApproximateTimeSynchronizer([self.rgb_subscriber, self.depth_subscriber],10,0.1)
        self.synchronyzer.registerCallback(self.geom_callback)

        self.Gcmap_publisher = self.create_publisher(Costmap, "/topic_G_cmap", 10)
        self.correspondance_publisher = self.create_publisher(Image, "/topic_G_grid", 10) #publishes the vector containing for each point its grid coordinates 
    
    # function to restructure the transformation matrix from the transform stamped message
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
        
        #self.get_logger().info("Transformation matrix recieved")
        return tf_matrix
    
    def pinhole2eucl_depth(self, pinhole_depth, cx, cy, f):
        u = np.zeros((pinhole_depth.shape[0],pinhole_depth.shape[1]))
        c = np.arange(pinhole_depth.shape[0])[:,None]
        u = u + c
        v = u.T

        # Compute the squared distances
        u_diff_sq = np.square(u - cx)
        v_diff_sq = np.square(v - cy)

        # Calculate the adjusted depth
        depth_eucl = (pinhole_depth * np.sqrt(u_diff_sq + v_diff_sq + f**2)) / f

        return depth_eucl


    def compute_angles(self, normal_vectors, vertical_axis):

        dot_products = np.dot(normal_vectors, vertical_axis)
        normals_norm = np.linalg.norm(normal_vectors, axis = 1) #obtain a vector of norms
        axis_norm = np.linalg.norm(vertical_axis)
        angles = np.arccos(np.divide(dot_products, normals_norm * axis_norm))

        return angles
    

    # Works both for RGB images and depth images
    def restructure_image(self, array, width, height, channels):
        if channels == 3:
            array = np.frombuffer(array, dtype=np.uint8)
            image = array.reshape((width, height, channels))

        elif channels == 1:
            array = np.frombuffer(array, dtype=np.uint16)
            image = array.reshape((width, height))
        
        return image
            

    
    def geometry_cmap(self,coordinates, angles, cells_number):
        coordinates_augmented = np.zeros((coordinates.shape[0], 3))
        coordinates_augmented[:, 0:2] = coordinates

        # Cost assignment
        soft_threshold = 30 * pi / 180
        threshold = 70 * pi/ 180 #70 deg in radians
        for id, angle in enumerate(angles):

            if angle <= soft_threshold:
                cost = 2/pi * angle

            elif  soft_threshold <= angle <= threshold:
                cost = 1/threshold * angle
            
            elif threshold < angle < (2*pi - threshold):
                cost = 1 # To be cautious max cost is assigned to points with this strange behaviour
            
            # Cost decreases when inclined in the other direction
            elif (2*pi - threshold) <= angle <= (2*pi - soft_threshold):
                cost = 1 - (angle - (2*pi - threshold))/threshold
            
            elif (2*pi - soft_threshold) <= angle <= 2*pi:
                cost = 1 - (angle - 3*pi/2)/(pi/2)
            
            
            coordinates_augmented[id, 2] = cost
        
        # Grid map creation
        cmap = np.zeros((cells_number, cells_number, 2))
        origin = np.array([np.min(coordinates[:,0]), np.min(coordinates[:,1])])
        dimx = np.max(coordinates[:,0]) - np.min(coordinates[:,0])
        dimy = np.max(coordinates[:,1]) - np.min(coordinates[:,1])

        resolution = min(dimx/cells_number, dimy/cells_number)

        normalized_coordinates = (coordinates - origin) / resolution
        pix_coordinates = np.floor(normalized_coordinates).astype(np.uint8) #to have pixel indices back, NB x,y = v,u coordinates
        
        #To ensure that all the pixels are in the limit of the map
        pix_coordinates = np.clip(pix_coordinates, 0, cells_number -1)
        
        limitx = cells_number -1

        wf_grid_correspondance = np.zeros((coordinates_augmented.shape[0],2)) #with the same order as the points coordinates stores the grid coordinates
        wf_grid_correspondance[:,0] = - pix_coordinates[:,1] + limitx #u coordinates, to have the right orientation
        wf_grid_correspondance[:,1] = pix_coordinates[:,0] #v coordinates

        # Bidimensional cost map
        for i, point in enumerate(pix_coordinates):
            u = limitx - point[1]
            v = point[0]
            cmap[u,v,0] += coordinates_augmented[i, 2]
            cmap[u,v,1] += 1
        
        # Average the costs in the map
        non_empty_cells = cmap[:,:,1] > 0
        cmap[non_empty_cells, 0] /= cmap[non_empty_cells, 1] #averages only the cells with more than one coordinate projected
        
        return cmap, wf_grid_correspondance
        


    def geom_callback(self, rgb:Image, depth:Image):
        # Application of geometry based approach
        # Retrieval of rgb and depth images from the messages

        # Creation cloud from depth and rgb
        rgb_image_data = rgb.data
        rgb_image = self.restructure_image(array=rgb_image_data, width=rgb.width, height=rgb.height, channels=3)

        depth_image_data = depth.data
        depth_image = self.restructure_image(array=depth_image_data, width=depth.width, height=depth.height, channels=1)
        
        # In uint16 the data are in mm, divide by 1000 to have meters
        depth_image = (depth_image / 1000).astype(np.float32)
        depth_image = self.pinhole2eucl_depth(pinhole_depth= depth_image, cx= self.cx, cy= self.cy, f= self.fx)
        depth_image = depth_image.astype(np.float32)



        rgb_component = open3d.geometry.Image(rgb_image)
        depth_component = open3d.geometry.Image(depth_image)
        rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(rgb_component, depth_component, depth_scale = 1.0, depth_trunc = 2000.0)


        point_cloud = open3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,self.camera)
        point_cloud.transform(self.preliminary_T) #to get standard camera orientation


        # Lookup of transformation matrix
        # Retry mechanism to avoid not having transformation available
        try_count = 0
        max_tries = 5
        tf_message = None

        while tf_message is None and try_count < max_tries:

            try:
                # Modified lookup which considers a tolerance in the acceptance of the result
                tf_message = self.tf_buffer.lookup_transform(self.wf, self.sf, rgb.header.stamp,
                                                        rclpy.duration.Duration(seconds = 1.0))
            except TransformException as ex:
                self.get_logger().info(f"Could not transform from {self.sf} to {self.wf}: {ex}")
                rclpy.spin_once(self, timeout_sec=0.1)
                try_count += 1
        
        if tf_message is None:
            self.get_logger().error(f"Failed to get the transformation after {max_tries} tries")
            return 
        

        tf_matrix = self.restructure_tf(tf_message) #restructure of the matrix from the message

        # tf matrix used to transform the cloud into world frame
        point_cloud.transform(tf_matrix)

        # estimation of normals of the cloud and correct re-orientation
        point_cloud.estimate_normals(
            search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        point_cloud.orient_normals_to_align_with_direction([0.0,0.0,1.0])
        
        normals_array = np.asarray(point_cloud.normals)

        # Computation of the angle between normals and vertical axis --> slope
        vertical = np.array([0,0,1])
        slopes = self.compute_angles(normal_vectors = normals_array, vertical_axis = vertical)

        # Creation of the geometry based cost map
        wf_coordinates = np.asarray(point_cloud.points)
        cmap, wf_grid_corresp = self.geometry_cmap(coordinates= wf_coordinates[:,0:2], angles= slopes, cells_number=256)
        cmap = cmap[:,:,0]
        #self.get_logger().info(f"Shape of the costmap: {cmap.shape}")
        
        # Map converted in the 0-255 range
        cmap = np.floor_divide(cmap, (1/255))
        cmap = cmap.astype(np.uint8)

        # Message creation
        cmap_msg = Costmap()
        grid_msg = Image()

        # Restructure the cmap since must be sent as a 1d array
        cmap_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.stamp = cmap_msg.header.stamp

        cmap_msg.header.frame_id = "odom"
        cmap_msg.metadata.size_x = cmap.shape[1] #number of columns of the map
        cmap_msg.metadata.size_y = cmap.shape[0] #number of rows of the map
        flattened_cmap = np.ndarray.flatten(cmap,'C')
        cmap_msg.data = flattened_cmap.tobytes()

        grid_msg.width = 1024
        grid_msg.height = 1024
        grid_msg.encoding = "mono8"
        grid_msg.step = grid_msg.width * 2 #2 grid coordinates
        flattened_data = np.ndarray.flatten(wf_grid_corresp, "C")
        grid_data = flattened_data.astype(np.uint8)
        grid_msg.data = grid_data.tobytes()

        # Publishing of the costmap and the grid coordinates
        self.Gcmap_publisher.publish(cmap_msg)
        self.correspondance_publisher.publish(grid_msg)
        data = grid_msg.data
        data_array = np.frombuffer(data, dtype=np.uint8)




        


def main(args = None):
    rclpy.init(args = args)
    node = geometry_node()
    rclpy.spin(node)
    rclpy.shutdown()

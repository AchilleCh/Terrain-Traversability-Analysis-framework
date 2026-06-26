#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import albumentations as A
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import Image
import numpy as np

class final_node(Node):
    def __init__(self):
        super().__init__("final_node")
        self.get_logger().info("final_node started")
        self.geom_cmap_sub = self.create_subscription(Costmap, "/topic_G_cmap_OUT", self.geom_callback, 10)
        self.app_cmap_sub = self.create_subscription(Costmap, "/topic_A_cmap_OUT", self.app_callback, 10)
        self.grid_geom_sub = self.create_subscription(Image, "/topic_G_grid_OUT", self.grid_geom_callback, 10) 
        self.wg = 0.5
        self.wa = 0.5
        self.app_cmap = np.empty((256,256))
        self.geom_cmap = np.empty((256,256))
        self.grid_corresp = np.empty((1024,1024,2))
        self.resize_coordinates = A.Compose([A.Resize(height=256,width=256),])
        self.interpolation_radius = 10
        self.flagA = False
        self.flagG = False
        self.flagC = False
        self.create_timer(0.1,self.final_cmap_callback)
        self.final_cmap_pub = self.create_publisher(Costmap,"/topic_final_cmap", 10)
    

    def restructure_map(self, width, height, array):
        
        array = np.frombuffer(array, dtype=np.uint8)
        matrix = array.reshape((width, height))

        return matrix


    def app_callback(self, msg:Costmap):
        width_app = msg.metadata.size_x
        height_app = msg.metadata.size_y
        app_cmap_array = msg.data

        self.app_cmap = self.restructure_map(width = width_app, height = height_app, array = app_cmap_array)
        self.flagA = True
    
    def geom_callback(self, msg:Costmap):
        width_geom = msg.metadata.size_x
        height_geom = msg.metadata.size_y
        geom_cmap_array = msg.data

        self.geom_cmap = self.restructure_map(width = width_geom, height = height_geom, array = geom_cmap_array)
        self.flagG = True

    def grid_geom_callback(self, msg:Image):
        width_grid = msg.width
        height_grid = msg.height
        channels = 2
        grid_data = msg.data
        grid_array = np.frombuffer(grid_data, dtype=np.uint8)
        grid_corresp = grid_array.reshape((width_grid, height_grid, channels))
        self.grid_corresp = grid_corresp
        self.flagC = True

    def interpolate_cell(self,cmap,ci,cj,radius):
        new_value = 0

        min_u = max(ci - radius, 0)
        max_u = min(ci + radius, 255)

        min_v = max(cj - radius, 0)
        max_v = min(cj + radius, 255)
        
        
        window = cmap[min_u:max_u+1, min_v:max_v+1]
        non_zero_points = window[window > 0]
        num_points = window.shape[0] * window.shape[1]
            


        print("Interpolation ended")
        if num_points>0:
            # Average only over the nonzero points in the window
            new_value = np.sum(window) / non_zero_points.size
        
        else:
            print("No points in the radius is admissible")

        return new_value
    
        
    def final_cmap_callback(self):
        if self.flagA == True and self.flagG == True and self.flagC == True:
            self.get_logger().info("Cost maps recieved")

            # Creation of the wf app based cmap
            #1) Resize of the grid correspondences:
            wf_app_cmap = np.zeros((256,256,2))
            grid_coordinates = self.resize_coordinates(image = self.grid_corresp/255)["image"]
            grid_coordinates = np.round(grid_coordinates * 255).astype(int)
            
            #2) Build the grid wf app cmap:
            for i, row in enumerate(grid_coordinates):
                for j, el in enumerate(row):
                   cost = self.app_cmap[i,j]
                   ug = grid_coordinates[i,j,0]
                   vg = grid_coordinates[i,j,1]
                   wf_app_cmap[ug,vg,0] += cost
                   wf_app_cmap[ug,vg,1] += cost

            #3) Average costs
            non_empty_cells = wf_app_cmap[:,:,1] > 0
            wf_app_cmap[non_empty_cells, 0] /= wf_app_cmap[non_empty_cells, 1] #averages only the cells with more than one point projected
            wf_app_cmap_values = np.copy(wf_app_cmap[:,:,0])

            #4) Interpolate cells to make the app cmap less sparse
            count = 1
            iter = 0
            while count > 0 and iter < 5:
                count_past = count
                count = 0 #to restart from blank every iteration
                for i, row in enumerate(self.geom_cmap):
                    for j, _ in enumerate(row):
                        if wf_app_cmap_values[i,j] == 0 and self.geom_cmap[i,j] > 0:
                            wf_app_cmap_values[i,j] = self.interpolate_cell(cmap=wf_app_cmap_values, ci=i, cj=j, radius=self.interpolation_radius)
                            count += 1 #add to the points to interpolate
                            if wf_app_cmap_values[i,j] > 0:
                                count = count - 1 #take out points for which interpolation was successful
                            

                self.get_logger().info(f"there are {count} cells empty in app cmap and not in geom cmap")
                if count == count_past:
                    iter += 1
                    self.interpolation_radius += 5 #if interpolation process fails retry 5 times increasing the radius by 5 pixels each time
                else:
                    iter = 0
            
            self.interpolation_radius = 10
            wf_app_cmap_values = np.floor_divide(wf_app_cmap_values, (1/255)).astype(np.uint8)
            

            # Creation of the hybrid map
            final_cmap = self.wg * self.geom_cmap + self.wa * wf_app_cmap_values


            # Creation of the final costmap message to be sent
            cmap_msg = Costmap()
            cmap_msg.header.stamp = self.get_clock().now().to_msg()
            cmap_msg.header.frame_id = "odom"
            cmap_msg.metadata.size_x = final_cmap.shape[1] #columns
            cmap_msg.metadata.size_y = final_cmap.shape[0] #rows
            final_cmap_array = np.ndarray.flatten(final_cmap, 'C')
            cmap_msg.data = final_cmap_array.tobytes()

            self.final_cmap_pub.publish(cmap_msg)  
            self.flagA = False
            self.flagG = False
            self.flagC = False
            self.get_logger().warning("Final traversability cost map published")      
            self.get_logger().info(f"shape of final map: {final_cmap.shape}")
  





def main(args = None):
    rclpy.init(args = args)
    node = final_node()
    rclpy.spin(node = node)
    rclpy.shutdown()
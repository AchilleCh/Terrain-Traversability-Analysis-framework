# README of ROS2 framework for Terrain Traversability Analysis:

## Nodes:

**- publisher_node.py:** Publishes images and point clouds

**- static transformer:** Camera odom transformation

**- appearance_based_node.py:** Appearance based approach, publishes wf cmap and grid correspondance 

**- geometry_based.py:** Geometry based approach, publishes cf cmap

**- filter2.py:** Synchronizer filter

**- final_node.py:** Creates wf app cmap, interpolates it and creates and publishes final cmap

**- map_converter:** Creates OccupancyGrid message from costmap2 message

**- framework_launch:** Launch file

## Utils:

Utils functions and setup.py and package.xml
##

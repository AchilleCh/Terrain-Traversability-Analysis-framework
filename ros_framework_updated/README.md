# README of ROS2 framework for Terrain Traversability Analysis:

## Nodes:

**- publisher_node.py:** Publishes images and point clouds
**- static transformer:** Camera odom transformation

**- appearance_based_node.py:** app based approach, publishes wf cmap and grid correspondance 
**- geometry_based.py:** geometry based approach, publishes cf cmap

**- filter2.py:** synchronizer filter
**- final_node.py:** creates wf app cmap, interpolates it and creates and publishes final cmap
**- map_converter:** creates OccupancyGrid message from costmap2 message

**- framework_launch:** launch file

## Utils:

Utils functions and setup.py and package.xml

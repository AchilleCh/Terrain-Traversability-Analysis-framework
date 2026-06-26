README:

nodes:

- publisher_node.py--> publishes images and point clouds
- static transformer --> camera odom transformation

- appearance_based_node.py--> app based approach, publishes wf cmap and grid correspondance 
- geometry_based.py--> geometry based approach, publishes cf cmap

- filter2.py--> synchronizer
- final_node.py--> creates wf app cmap, interpolates it and creates and publishes final cmap
- map_converter --> creates OccupancyGrid message from costmap2 message

- framework_launch --> launch file

NB:
checkpoints files for the neural networks are missing

utils:

utils functions and setup.py and package.xml

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    pub_node =         Node(
            package="ros_framework",
            executable="publisher_node"
        )
    
    filter1 =         Node(
            package="ros_framework",
            executable="filter_node1"
        )
    
    app_based_node =         Node(
            package="ros_framework",
            executable="appearance_node"
        )
    
    geom_based_node =         Node(
            package="ros_framework",
            executable="geometry_node"
        )
    
    filter2 =         Node(
            package="ros_framework",
            executable="filter_node2"
        )
    
    final_node =         Node(
            package="ros_framework",
            executable="final_node"
        )
    
    map_converter = Node(
            package="ros_framework",
            executable="map_converter_node"    )
    
    camera_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "1", "1", "0", "0", "0.32288591161895097", "odom", "camera"]) #-->[tx,ty,tz,y[rad],p,r,target_frame,source_frame] from ros doc, verify
    

    return LaunchDescription([

        pub_node,
        camera_tf,
        app_based_node,
        geom_based_node,
        filter2,
        final_node,
        map_converter,

    ])
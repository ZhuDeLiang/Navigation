#!/bin/bash
rosservice call /write_state "filename: '/home/$USER/catkin_ws/src/agv_robot/maps/map.pbstream'"
# ros agv_robot cartographer_pbstream_to_ros_map.launch map_filestem:=/home/$USER/catkin_ws/src/agv_robot/maps/map pbstream_filename:=/home/$USER/catkin_ws/src/agv_robot/maps/map.pbstream resolution:=0.05
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/$USER/catkin_ws/src/agv_robot/maps/map -pbstream_filename=/home/$USER/catkin_ws/src/agv_robot/maps/map.pbstream -resolution=0.05
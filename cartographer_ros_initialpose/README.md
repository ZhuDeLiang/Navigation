# cartographer_ros_initialpose  
A package that reset the initial pose of the robot using cartographer for slam or navigation. This make cartographer_ros usable for pure localization like amcl.
## Subscribe  
* initialpose  
The initlal pose to set. Usually this topic is published from RViz.
## Service  
* cartographer_ros_initialpose  
A simple service server for setting initial pose.
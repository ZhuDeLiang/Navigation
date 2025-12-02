#include <stdlib.h>
#include <ros/ros.h>
#include <cartographer_ros_initialpose/initial_pose.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cartographer_ros_initialpose");
    if(argc!=3)
    {
        ROS_ERROR("You must set configuration_directory and configuration_basename.");
        return -1;
    }
    cartographer_ros::InitialPose initial_pose_client(argv[1],argv[2]);
    ros::spin();
    return 0;
}
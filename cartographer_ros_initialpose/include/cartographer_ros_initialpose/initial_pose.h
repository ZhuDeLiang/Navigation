#pragma once
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <turtlesim/Spawn.h>

namespace cartographer_ros
{
/**
* This class implements reset the pose of robot using cartographer_ros(1.0) for localization.
*/
class InitialPose
{
public:
    /**
    * @brief Constructor
    */
    InitialPose(std::string dir,std::string config_file);
private:
    /**
    * @brief Function that sets a new initial pose
    * @param pose The initial pose to be set.
    */
    bool setInitialPose(const geometry_msgs::Pose& pose);
    /**
    * @brief Topic callback function
    */
    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr);
    /**
    * @brief Service callback function
    */
    bool initialPoseSrvCB(turtlesim::Spawn::Request& rq,turtlesim::Spawn::Response& rs);
    ros::NodeHandle m_nh_private{"~"};
    ros::NodeHandle m_nh;
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener{m_tf_buffer};
    std::string m_dir;//lua file directory
    std::string m_config_file;//lua file name
    std::string tracking_frame_{"imu_link"};
    int m_traj_id{1};//trajectory id
    ros::Subscriber m_initialpose_sub{m_nh.subscribe("initialpose",1,&cartographer_ros::InitialPose::initialPoseCB,this)};//<geometry_msgs::PoseWithCovarianceStamped,cartographer_ros::InitialPose>
    ros::ServiceClient m_finish_traj_client{m_nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory")};//service client to finish the last trajectory
    ros::ServiceServer m_initialpose_srv{m_nh.advertiseService("cartographer_ros_initialpose",&cartographer_ros::InitialPose::initialPoseSrvCB,this)};//<cartographer_ros::InitialPose,turtlesim::Spawn::Request,turtlesim::Spawn::Response>
};
}

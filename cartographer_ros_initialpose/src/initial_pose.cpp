#include <cartographer_ros_initialpose/initial_pose.h>
#include <stdlib.h>
#include <sstream>
#include <tf2/utils.h>

namespace cartographer_ros
{
InitialPose::InitialPose(std::string dir,std::string config_file):m_dir(dir),m_config_file(config_file)
{
    std::cout<<"cartographer_ros configure file: "<<m_dir<<std::string(m_dir.back()=='/'?"":"/")<<m_config_file<<std::endl;
    m_nh_private.param("tracking_frame",tracking_frame_,std::string("imu_link"));
}

void InitialPose::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
    if(setInitialPose(ptr->pose.pose))
        ROS_INFO("Initial pose has been set.");
    else
        ROS_ERROR("An error occered when setting initial pose.");
    return;
}

bool InitialPose::setInitialPose(const geometry_msgs::Pose& pose)
{
    cartographer_ros_msgs::FinishTrajectory finish_srv_msg;
    geometry_msgs::TransformStamped Tbi_msg;
    try
    {
        Tbi_msg=m_tf_buffer.lookupTransform("base_link",tracking_frame_,ros::Time(1));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }
    while(ros::ok())
    {
        finish_srv_msg.request.trajectory_id=m_traj_id;
        if(m_finish_traj_client.call(finish_srv_msg))
        {
            std::cout<<finish_srv_msg.response.status.message<<std::endl;
            if(finish_srv_msg.response.status.code==0)//Finished trajectory 0
                break;
            else if(m_traj_id==1&&finish_srv_msg.response.status.code==5)
            {
                m_traj_id--;
                break;
            }
            else if(finish_srv_msg.response.status.code==5)//Trajectory is not created yet 5
            {
                std::cout<<m_traj_id<<" Reset trajectory id."<<std::endl;
                m_traj_id=1;
            }
            else//Trajectory has already been finished 8, Trajectory is frozen 3
                std::cout<<m_traj_id<<" Try next trajectory."<<std::endl;
        }
        else
        {
            ROS_ERROR("Fail to call finish_trajectory service");
            m_traj_id=1;
            return false;
        }
        m_traj_id++;
    }
    m_traj_id++;
    geometry_msgs::Pose pose_bi;
    pose_bi.position.x=Tbi_msg.transform.translation.x;
    pose_bi.position.y=Tbi_msg.transform.translation.y;
    pose_bi.position.z=Tbi_msg.transform.translation.z;
    pose_bi.orientation=Tbi_msg.transform.rotation;
    tf2::Transform Tmb,Tbi;
    tf2::fromMsg(pose,Tmb);
    tf2::fromMsg(pose_bi,Tbi);
    tf2::Transform Tmi=Tmb*Tbi;
    std::string start_traj_cmd=std::string("rosrun cartographer_ros cartographer_start_trajectory -configuration_directory ");
    start_traj_cmd+=m_dir;
    start_traj_cmd+=" -configuration_basename ";
    start_traj_cmd+=m_config_file;
    start_traj_cmd+=" -initial_pose '{to_trajectory_id=0, timestamp=0, relative_pose={translation={";
    std::stringstream ss;
    ss<<Tmi.getOrigin().x()<<","<<Tmi.getOrigin().y()<<","<<Tmi.getOrigin().z()<<"},rotation={"<<0<<","<<0<<","<<tf2::getYaw(Tmi.getRotation())<<"}}}'";
    std::string pose_str;
    ss>>pose_str;
    start_traj_cmd+=pose_str;
    ROS_INFO("%s",start_traj_cmd.c_str());
    auto result=system(start_traj_cmd.c_str());
    return result==0?true:false;
}

bool InitialPose::initialPoseSrvCB(turtlesim::Spawn::Request& rq,turtlesim::Spawn::Response& rs)
{
    geometry_msgs::Pose msg;
    tf2::Quaternion tf2q;
    tf2q.setRPY(0.0,0.0,static_cast<double>(rq.theta));
    msg.position.x=static_cast<double>(rq.x);
    msg.position.y=static_cast<double>(rq.y);
    msg.orientation=tf2::toMsg(tf2q);
    auto result=setInitialPose(msg);
    if(result)
        rs.name=std::string("succeeded");
    else
        rs.name=std::string("failded");
    return result;
}
}//end namespace

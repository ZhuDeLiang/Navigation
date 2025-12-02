//解决设置目标点的时候自动清除代价地图和设置启动初始位置
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "tf/tf.h"
geometry_msgs::PoseStamped a;
geometry_msgs::PoseWithCovarianceStamped initial_pose;
ros::ServiceClient client;
std_srvs::Empty clear_costmap_srv;
double yaw;
bool use_clear_costmap = true;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
int main(int argc, char** argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  ros::Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, callback);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  nh_local.param("position_x", initial_pose.pose.pose.position.x, (double)0.0);
  nh_local.param("position_y", initial_pose.pose.pose.position.y, (double)0.0);
  nh_local.param("yaw", yaw, (double)0.0);
  nh_local.param("use_clear_costmap", use_clear_costmap, (bool)true);
  initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  ROS_INFO_STREAM("AMCL Initial Pose: "
                  << initial_pose.pose.pose.position.x << ", "
                  << initial_pose.pose.pose.position.y << ", " << yaw);

  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = ros::Time::now();
  sleep(1);
  pub.publish(initial_pose);
  ros::spin();
  return 0;
}
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  bool succ;
  if (use_clear_costmap) succ = client.call(clear_costmap_srv);
  if (succ)
    ROS_INFO_STREAM("Clear costmap.");
  else
    ROS_INFO_STREAM("Clear costmap failed.");
}
#ifndef AGV_ROBOT_NAV_CORE_H
#define AGV_ROBOT_NAV_CORE_H

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <csignal>
#include <cstdlib>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "actionlib/client/simple_action_client.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "ros/ros.h"
#include "ros/time.h"
// #include "rubber_navigation/BaseController.h"
#include "std_srvs/Empty.h"

class NavCore {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      MoveBaseClient;

 public:
  enum MoveBaseActionResult {
    PENDING,
    ACTIVE,
    PREEMPTED,
    SUCCEEDED,
    ABORTED,
    REJECTED,
    PREEMPTING,
    RECALLING,
    RECALLED,
    LOST,
    EMPTY,
  };

 private:
  const std::string BASE_FOOT_PRINT_;
  const std::string MAP_FRAME_;

  mutable boost::shared_mutex action_result_mutex_{};
  ros::NodeHandle nh;
  ros::Subscriber action_result_sub;

  bool isMoveBaseClientConnected_{};
  std::shared_ptr<MoveBaseClient> moveBaseClient;
  //    MoveBaseClient *moveBaseClient;
  MoveBaseActionResult moveBaseActionResult_{EMPTY};

  std_srvs::Empty clear_costmap_srv_;
  ros::ServiceClient client;

  geometry_msgs::Pose2D current_pose_{};
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  //   tf2_ros::TransformListener *tfListener_;

  void actionResultCallback(const move_base_msgs::MoveBaseActionResult &msg);

 public:
  NavCore(std::string base_foot_print, std::string map_frame);

  ~NavCore();

  bool isMoveBaseClientConnected() const { return isMoveBaseClientConnected_; };
  bool clearCostMap();
  void cancelAllGoals();

  void setGoal(const geometry_msgs::Pose2D &goal2d);

  MoveBaseActionResult getMoveBaseActionResult() {
    boost::unique_lock<boost::shared_mutex> writeLock(action_result_mutex_);
    MoveBaseActionResult temp{moveBaseActionResult_};
    moveBaseActionResult_ = MoveBaseActionResult::EMPTY;
    return temp;
  };

  const geometry_msgs::Pose2D &getCurrentPose(const std::string &target_frame,
                                              const std::string &source_frame);

  bool isGoalPassed(const geometry_msgs::Pose2D &goal_pose);
};

#endif  // AGV_ROBOT_NAV_CORE_H
#include "nav_core.h"
NavCore::NavCore(std::string base_foot_print, std::string map_frame)
    : BASE_FOOT_PRINT_(std::move(base_foot_print)),
      MAP_FRAME_(std::move(map_frame)) {
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
  ROS_INFO_STREAM("Connecting move_base ActionServer...");
  moveBaseClient = std::make_shared<MoveBaseClient>("move_base", true);
  //   moveBaseClient = new MoveBaseClient("move_base", true);
  moveBaseClient->waitForServer(ros::Duration(4.0));
  ros::NodeHandle nh_("~");
  if (!(isMoveBaseClientConnected_ = moveBaseClient->isServerConnected())) {
    nh_.setParam("isInitialized", (bool)false);
    ROS_ERROR_STREAM(
        "MoveBase ActionServer Failed， only telecontrol is enabled.");
  } else {
    nh_.setParam("isInitialized", (bool)true);
    ROS_INFO_STREAM("MoveBase ActionServer Connected, ready to navigate.");
  }

  client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  action_result_sub = nh.subscribe("/move_base/result", 10,
                                   &NavCore::actionResultCallback, this);
}
NavCore::~NavCore() {
  //   delete moveBaseClient;
  //   delete tfListener_;
}
void NavCore::actionResultCallback(
    const move_base_msgs::MoveBaseActionResult &msg) {
  boost::unique_lock<boost::shared_mutex> writeLock(action_result_mutex_);
  switch (msg.status.status) {
    case move_base_msgs::MoveBaseActionResult::_status_type::PENDING:
      moveBaseActionResult_ = MoveBaseActionResult::PENDING;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::ACTIVE:
      moveBaseActionResult_ = MoveBaseActionResult::ACTIVE;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::PREEMPTED:
      moveBaseActionResult_ = MoveBaseActionResult::PREEMPTED;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::SUCCEEDED:
      moveBaseActionResult_ = MoveBaseActionResult::SUCCEEDED;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::ABORTED:
      moveBaseActionResult_ = MoveBaseActionResult::ABORTED;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::REJECTED:
      moveBaseActionResult_ = MoveBaseActionResult::REJECTED;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::PREEMPTING:
      moveBaseActionResult_ = MoveBaseActionResult::PREEMPTING;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::RECALLING:
      moveBaseActionResult_ = MoveBaseActionResult::RECALLING;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::RECALLED:
      moveBaseActionResult_ = MoveBaseActionResult::RECALLED;
      break;
    case move_base_msgs::MoveBaseActionResult::_status_type::LOST:
      moveBaseActionResult_ = MoveBaseActionResult::LOST;
      break;
    default:
      moveBaseActionResult_ = MoveBaseActionResult::EMPTY;
      break;
  }
}
const geometry_msgs::Pose2D &NavCore::getCurrentPose(
    const std::string &target_frame, const std::string &source_frame) {
  geometry_msgs::TransformStamped transformStamped{};
  try {
    transformStamped = tfBuffer_.lookupTransform(
        target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  current_pose_.x = transformStamped.transform.translation.x;
  current_pose_.y = transformStamped.transform.translation.y;
  current_pose_.theta = tf::getYaw(transformStamped.transform.rotation);
  return current_pose_;
}
bool NavCore::isGoalPassed(const geometry_msgs::Pose2D &goal_pose) {
  getCurrentPose(MAP_FRAME_, BASE_FOOT_PRINT_);
  Eigen::Vector2d direction_array(cos(current_pose_.theta),
                                  sin(current_pose_.theta));

  Eigen::Vector2d distance_array(goal_pose.x - current_pose_.x,
                                 goal_pose.y - current_pose_.y);

  double distance = distance_array.dot(direction_array);

  return std::abs(distance) < 0.2 || distance < 0;
}
void NavCore::setGoal(const geometry_msgs::Pose2D &goal2d) {
  move_base_msgs::MoveBaseGoal goal{};
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal2d.x;
  goal.target_pose.pose.position.y = goal2d.y;
  goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(goal2d.theta);
  moveBaseClient->sendGoal(goal);
  ROS_INFO_STREAM("Set Goal(x,y,theta): " << goal2d.x << ", " << goal2d.y
                                          << ", " << goal2d.theta);
}
void NavCore::cancelAllGoals() {
  moveBaseClient->cancelAllGoals();
  ROS_INFO_STREAM("All goal canceled.");
}
bool NavCore::clearCostMap() {
  return client.call(clear_costmap_srv_);
  ROS_INFO_STREAM("Clear costmap.");
}

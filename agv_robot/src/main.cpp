#include "agv_robot/agv_cmd.h"
#include "control_teleop.h"
#include "nav_core.h"
#include "ros/ros.h"
#include "util.h"

class BaseNav {
  enum { DEFAULT, SET_GOAL, SET_POSE } CMD;
  // 01 正常状态，开始导航
  // 02 正常状态，到达目标点，成功
  // 03 异常状态，导航至目标点失败，放弃
  // 04 状态，导航至目标点过程中暂停导航，放弃
  // 05 正常状态，空闲状态下暂停导航
  // 06 正常状态，04情况下继续，接近目标点，成功
  // 07 正常状态，04情况下继续，未接近目标点，开始导航
  // 08 正常状态，空闲状态下继续导航
  // 09 异常状态，暂停状态下发送新目标点，放弃

 private:
  ros::NodeHandle nh_;
  DataWithMutex<bool> navOn, navPause, newGoal;
  std::shared_ptr<NavCore> navCore_;
  std::shared_ptr<CONTROLTELEOP::ControlTeleop> controlTeleop_;
  ros::Publisher statusPub_;
  ros::Subscriber cmdSub_;
  geometry_msgs::Pose2D targetPose_;
  void setBySignal();
  void logCallback(const agv_robot::agv_cmd::ConstPtr &msg);
  void pulishLog();
  std::string base_foot_print_, map_frame_;

 public:
  BaseNav(const std::string &base_foot_print, const std::string &odom_frame,
          const std::string &map_frame, bool publish_tf,
          const double max_joy_linear_velocity,
          const double max_joy_angular_velocity);
  ~BaseNav();
  void run();
};
BaseNav::BaseNav(const std::string &base_foot_print,
                 const std::string &odom_frame, const std::string &map_frame,
                 bool publish_tf, const double max_joy_linear_velocity,
                 const double max_joy_angular_velocity)
    : navOn(false),
      navPause(false),
      newGoal(false),
      base_foot_print_(base_foot_print),
      map_frame_(map_frame) {
  navCore_ = std::make_shared<NavCore>(base_foot_print, map_frame);
  controlTeleop_ = std::make_shared<CONTROLTELEOP::ControlTeleop>(
      true, max_joy_linear_velocity, max_joy_linear_velocity, false);
  statusPub_ = nh_.advertise<agv_robot::agv_cmd>("navi_status", 10);
  cmdSub_ = nh_.subscribe<agv_robot::agv_cmd>("navi_cmd", 10,
                                              &BaseNav::logCallback, this);
}

BaseNav::~BaseNav() {}

void BaseNav::logCallback(const agv_robot::agv_cmd::ConstPtr &msg) {
  if (msg->command == 3) {
    // 暂停导航时不接受目标点
    if (navPause == true) {
      ROS_WARN("Target received, but navigation is paused.");
      targetPose_ = msg->pose;
      agv_robot::agv_cmd feedback;
      feedback.nav_status = 9;
      statusPub_.publish(feedback);
      return;
    }
    targetPose_ = msg->pose;
    navCore_->clearCostMap();
    navCore_->setGoal(targetPose_);
    newGoal = true;
    agv_robot::agv_cmd feedback;
    feedback.nav_status = 1;
    ROS_INFO("Sart navigation.");
    statusPub_.publish(feedback);
  } else if (msg->command == 1) {
    if (navPause) {
      // 已经到目标点附近，成功
      if (navCore_->isGoalPassed(targetPose_)) {
        agv_robot::agv_cmd feedback;
        feedback.nav_status = 6;
        newGoal = false;
        statusPub_.publish(feedback);
        ROS_INFO_STREAM(
            "Navigation resumed, but distance to the target is less than 0.2m, "
            "goal reached.");
        return;
      }
      // 未到目标点附近，继续当前目标
      navCore_->clearCostMap();
      navCore_->setGoal(targetPose_);
      ROS_INFO("Navigation resumed.");
      agv_robot::agv_cmd feedback;
      feedback.nav_status = 7;
      statusPub_.publish(feedback);
      newGoal = true;
    }
    // 非暂停状态下，允许接受目标点
    navOn = true;
    navPause = false;
    agv_robot::agv_cmd feedback;
    feedback.nav_status = 8;
    statusPub_.publish(feedback);
    ROS_INFO("NavOn received, ready to go");
  } else if (msg->command == 2) {
    navOn = false;
    navPause = true;
    // 导航中暂停，取消导航任务
    if (newGoal) {
      ROS_INFO("Navigation paused, cancel all goals");
      navCore_->cancelAllGoals();
      agv_robot::agv_cmd feedback;
      feedback.nav_status = 4;
      statusPub_.publish(feedback);
    }
    // 空闲状态暂停，拒绝新目标点
    else {
      ROS_INFO("Navigation paused, new goal won't be executed.");
      agv_robot::agv_cmd feedback;
      feedback.nav_status = 5;
      statusPub_.publish(feedback);
    }
  }
}
void BaseNav::setBySignal() {
  switch (controlTeleop_->getControlTrigger()) {
    // 继续导航
    case CONTROLTELEOP::NavOn: {
      // 暂停状态下
      if (navPause) {
        // 已经到目标点附近，成功
        if (navCore_->isGoalPassed(targetPose_)) {
          agv_robot::agv_cmd feedback;
          feedback.nav_status = 6;
          newGoal = false;
          ROS_INFO_STREAM(
              "Navigation resumed, but distance to the target is less than "
              "0.2m, "
              "goal reached.");
          statusPub_.publish(feedback);
          break;
        }
        // 未到目标点附近，继续当前目标
        navCore_->clearCostMap();
        navCore_->setGoal(targetPose_);
        ROS_INFO("Navigation resumed.");
        agv_robot::agv_cmd feedback;
        feedback.nav_status = 7;
        statusPub_.publish(feedback);
        newGoal = true;
      }
      // 非暂停状态下，允许接受目标点
      navOn = true;
      navPause = false;
      agv_robot::agv_cmd feedback;
      feedback.nav_status = 8;
      statusPub_.publish(feedback);
      ROS_INFO("NavOn received, ready to go");
      break;
    }
    case CONTROLTELEOP::NavPause: {
      navOn = false;
      navPause = true;
      // 导航中暂停，取消导航任务
      if (newGoal) {
        ROS_INFO("Navigation paused, cancel all goals");
        navCore_->cancelAllGoals();
        agv_robot::agv_cmd feedback;
        feedback.nav_status = 4;
        statusPub_.publish(feedback);
      }
      // 空闲状态暂停，拒绝新目标点
      else {
        ROS_INFO("Navigation paused, new goal won't be executed.");
        agv_robot::agv_cmd feedback;
        feedback.nav_status = 5;
        statusPub_.publish(feedback);
      }
      break;
    }
    default:
      break;
  }
}
void BaseNav::run() {
  setBySignal();
  agv_robot::agv_cmd status;
  status.pose = navCore_->getCurrentPose(base_foot_print_, map_frame_);
  switch (navCore_->getMoveBaseActionResult()) {
    case NavCore::SUCCEEDED: {
      newGoal = false;
      status.nav_status = 2;
      status.text = "06 Current goal REACHED.";
      break;
    }
    case NavCore::ABORTED: {
      newGoal = false;
      status.nav_status = 3;
      status.text = "07 Current goal ABORTED.";
      break;
    }
    default:
      break;
  }
  statusPub_.publish(status);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "base_navigation");
  ros::NodeHandle nh_("~");
  ros::Rate loop_rate(30);
  std::string base_foot_print, odom_frame, map_frame;
  double max_joy_linear_velocity, max_joy_angular_velocity;
  bool publish_tf;
  nh_.param("base_foot_print", base_foot_print, (std::string) "base_link");
  nh_.param("odom_frame", odom_frame, (std::string) "odom");
  nh_.param("map_frame", map_frame, (std::string) "map");
  nh_.param("publish_tf", publish_tf, (bool)false);
  nh_.param("max_joy_linear_velocity", max_joy_linear_velocity, (double)0.5);
  nh_.param("max_joy_angular_velocity", max_joy_angular_velocity, (double)1.0);
  ros::AsyncSpinner spinner(5);
  spinner.start();
  BaseNav baseNav(base_foot_print, odom_frame, map_frame, publish_tf,
                  max_joy_linear_velocity, max_joy_angular_velocity);
  while (ros::ok()) {
    baseNav.run();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
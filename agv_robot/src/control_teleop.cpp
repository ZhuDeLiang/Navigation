#include "control_teleop.h"
using namespace CONTROLTELEOP;

ControlTeleop::ControlTeleop(bool publish_vel, double max_linear_velocity,
                             double max_angular_velocity, bool diff_drive) {
  control_trigger_ = Default;
  publish_vel_ = publish_vel;
  diff_drive_ = diff_drive;
  Joy_sub_ = nh_.subscribe("joy", 100, &ControlTeleop::JoyCallback, this);
  if (publish_vel) {
    max_angular_velocity_ = max_angular_velocity;
    max_linear_velocity_ = max_linear_velocity;
    joy_vel_pub = nh_.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    ROS_INFO_STREAM(
        "Enable joy control, velocity commands are published in topic: "
        << joy_vel_pub.getTopic());
    ROS_INFO_STREAM("Max linear vel:  " << max_linear_velocity_);
    ROS_INFO_STREAM("Max angular vel: " << max_angular_velocity_);
    if (diff_drive_)
      ROS_INFO_STREAM("Using differential drive.");
    else
      ROS_INFO_STREAM("Using omnidirectional drive.");
  }
  joy_watchdog_timer_ = nh_.createTimer(
      ros::Duration(WATCHDOG_PERIOD_), &ControlTeleop::Joywatchdog, this, true);
  joy_watchdog_timer_.start();
}

void ControlTeleop::Joywatchdog(const ros::TimerEvent &e) {
  // ROS_WARN("joy not received for %f seconds, is the joy node drop?",
  //          WATCHDOG_PERIOD_);
  this->joy_alive_ = false;
}
void ControlTeleop::JoyCallback(const sensor_msgs::JoyConstPtr &msg) {
  joy_watchdog_timer_.stop();
  joy_watchdog_timer_.start();
  boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
  if (msg->axes[2] == -1 && msg->buttons[0])  // A
  {
    control_trigger_ = ManipulateOn;
  } else if (msg->axes[2] == -1 && msg->buttons[1])  // B
  {
    control_trigger_ = ChargeOn;
  } else if (msg->axes[2] == -1 && msg->buttons[2])  // X
  {
    control_trigger_ = HomeOn;
  } else if (msg->axes[2] == -1 && msg->buttons[3])  // Y
  {
    control_trigger_ = UpOn;
  } else if (msg->axes[2] == -1 && msg->axes[6] == 1) {
    control_trigger_ = LinearBack;
  } else if (msg->axes[2] == -1 && msg->axes[6] == -1) {
    control_trigger_ = LinearForward;
  } else if (msg->axes[2] == -1 && msg->axes[7] == 1) {
    control_trigger_ = LinearUp;
  } else if (msg->axes[2] == -1 && msg->axes[7] == -1) {
    control_trigger_ = LinearDown;
  } else if (msg->axes[2] == -1 && msg->buttons[5])  //
  {
    control_trigger_ = ArmEmergencyChange;
  } else if (msg->axes[2] == -1 && msg->buttons[4]) {
    control_trigger_ = CutBack;
  } else if (msg->axes[2] == -1 && msg->axes[5] == -1) {
    control_trigger_ = KnifeUnplug;
  }
  /*else if(msg->axes[2]==-1&&msg->axes[7]==1)//up
  {
      control_trigger_ = LightOn;
  }
  else if(msg->axes[2]==-1&&msg->axes[7]==-1)//down
  {
      control_trigger_ = LightOff;
  }*/
  else if (msg->axes[5] == -1 && msg->buttons[0])  // A
  {
    control_trigger_ = KnifeOff;
  } else if (msg->axes[5] == -1 && msg->buttons[1])  // B
  {
    control_trigger_ = ClockGo;
  } else if (msg->axes[5] == -1 && msg->buttons[2])  // X
  {
    control_trigger_ = AntiClockGo;
  } else if (msg->axes[5] == -1 && msg->axes[6] == 1)  // left
  {
    control_trigger_ = SingleAntiClockGo;
  } else if (msg->axes[5] == -1 && msg->axes[6] == -1)  // right
  {
    control_trigger_ = SingleClockGo;
  } else if (msg->axes[5] == -1 && msg->buttons[3])  // Y
  {
    control_trigger_ = KnifeOn;
  } else if (msg->axes[5] == -1 && msg->buttons[10]) {
    control_trigger_ = ShutDown;
  } else if (msg->axes[5] == -1 && msg->buttons[4]) {
    control_trigger_ = SteeringIn;
  } else if (msg->axes[5] == -1 && msg->buttons[5]) {
    control_trigger_ = SteeringOut;
  } else if (msg->buttons[0]) {
    control_trigger_ = SaveCutPoint;
  } else if (msg->buttons[1]) {
    control_trigger_ = SaveChargePoint;
  } else if (msg->buttons[2]) {
    control_trigger_ = SaveNavPoint;
  } else if (msg->buttons[3]) {
    control_trigger_ = SaveTurnPoint;
  } else if (msg->buttons[4])  // L1
  {
    control_trigger_ = NavOn;
  } else if (msg->buttons[5])  // R1
  {
    control_trigger_ = NavPause;
  } else if (msg->buttons[6]) {
    control_trigger_ = MappingOff;
  } else if (msg->buttons[7]) {
    control_trigger_ = MappingOn;
  } else if (msg->buttons[9]) {
    control_trigger_ = RobotArmOn;
  } else if (msg->buttons[10]) {
    control_trigger_ = NavigationOn;
  } else
    control_trigger_ = Default;

  if (publish_vel_) {
    geometry_msgs::Twist vel{};
    if (diff_drive_) {
      if (msg->axes[0] || msg->axes[1]) {
        vel.angular.z = msg->axes[0] * max_angular_velocity_;
        vel.linear.x = msg->axes[1] * max_linear_velocity_;
        if (msg->axes[3] || msg->axes[4]) {
          vel.angular.z = msg->axes[0] * max_angular_velocity_ +
                          msg->axes[3] * max_angular_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
        }
      } else if (msg->axes[3] || msg->axes[4]) {
        vel.angular.z = msg->axes[3] * max_angular_velocity_ / 2.0;
        vel.linear.x = msg->axes[4] * max_linear_velocity_ / 2.0;
        if (msg->axes[0] || msg->axes[1]) {
          vel.angular.z = msg->axes[0] * max_angular_velocity_ +
                          msg->axes[3] * max_angular_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
        }
      }
    } else {
      if (msg->axes[0] || msg->axes[1] || msg->axes[2] != 1 ||
          msg->axes[5] != 1) {
        vel.linear.y = msg->axes[0] * max_linear_velocity_;
        vel.linear.x = msg->axes[1] * max_linear_velocity_;
        vel.angular.z =
            (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        if (msg->axes[3] || msg->axes[4]) {
          vel.linear.y = msg->axes[0] * max_linear_velocity_ +
                         msg->axes[3] * max_linear_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
          vel.angular.z =
              (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        }
      } else if (msg->axes[3] || msg->axes[4]) {
        vel.linear.y = msg->axes[3] * max_linear_velocity_ / 2.0;
        vel.linear.x = msg->axes[4] * max_linear_velocity_ / 2.0;
        vel.angular.z =
            (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        if (msg->axes[0] || msg->axes[1]) {
          vel.linear.y = msg->axes[0] * max_linear_velocity_ +
                         msg->axes[3] * max_linear_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
          vel.angular.z =
              (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        }
      }
    }
    joy_vel_pub.publish(vel);
  }
}
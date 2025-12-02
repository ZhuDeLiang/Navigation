#ifndef AGV_ROBOT_TCP_CLIENT_H
#define AGV_ROBOT_TCP_CLIENT_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <geometry_msgs/Twist.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <signal.h>

#include <iostream>

#include "agv_robot/agv_cmd.h"
#include "protocol.h"
#include "util.h"
class TCPClient {
 private:
  enum { DEFAULT, IDLE, NAV, PAUSE } STATUS;
  NaviCmd cmd_;
  NaviStatus navi_status;
  int err_code;
  bool isMapExist{false};
  bool poseValid{false};
  ros::Time cmd_vel_watch;
  DataWithMutex<bool> cmd_vel_received = false;
  DataWithMutex<bool> joy_vel_received = false;
  unsigned char velocity_[12]{0x00};
  unsigned char pose[12]{0x00};
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber joy_vel_sub;
  ros::Publisher cmd_pub;

  std::string server_ip;
  std::string map_name;
  int port;

  int cmd_size = sizeof(cmd_);
  int status_size = sizeof(navi_status);

 private:
  void float2uchar(float fpointer, unsigned char* a);
  void uchar2float(unsigned char* a, float& fpointer);
  void status_callback(const agv_robot::agv_cmd::ConstPtr& msg);
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void joy_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void getRobotStatus();

 public:
  TCPClient(ros::NodeHandle& nh);
  void initSocket();
  void loop();
  static int getSockfd() { return sockfd_; }
  unsigned short getCRC16(const unsigned char* ptr, unsigned char len);
  static int sockfd_;
};
#endif
#include "tcp_client.h"

#include <std_msgs/String.h>
int TCPClient::sockfd_ = -1;
TCPClient::TCPClient(ros::NodeHandle& nh) : nh_(nh) {
  cmd_sub = nh_.subscribe<agv_robot::agv_cmd>(
      "navi_status", 10, &TCPClient::status_callback, this);
  cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &TCPClient::cmd_vel_callback, this);
  joy_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
      "joy_vel", 10, &TCPClient::joy_vel_callback, this);
  // cmd_pub = nh_.advertise<agv_robot::agv_cmd>("navi_cmd", 10);
  cmd_pub = nh_.advertise<agv_robot::agv_cmd>("navi_cmd", 10);

  ros::NodeHandle nh_local("~");
  nh_local.param("server_ip", server_ip, (std::string) "192.168.0.112");
  nh_local.param("port", port, (int)9054);
  nh_local.param(
      "map_name", map_name,
      (std::string) "/home/nurse/catkin_ws/src/agv_robot/maps/map.pgm");
  ROS_INFO_STREAM("TCP Server IP: " << server_ip);
  ROS_INFO_STREAM("COM Port： " << port);
  initSocket();
}
void TCPClient::initSocket() {
  struct hostent* he;
  /* 连接者的主机信息 */
  struct sockaddr_in their_addr;

  /* 取得主机信息 */
  if ((he = gethostbyname(server_ip.c_str())) == NULL) {
    /* 如果gethostbyname()发生错误，则显示错误信息并退出 */
    herror("gethostbyname");
    exit(1);
  }

  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    /* 如果socket()调用出现错误则显示错误信息并退出 */
    perror(" socket ");
    exit(1);
  }

  /* 主机字节顺序 */
  their_addr.sin_family = AF_INET;
  /* 网络字节顺序，短整型 */
  their_addr.sin_port = htons(port);

  their_addr.sin_addr = *((struct in_addr*)he->h_addr);
  /* 将结构剩下的部分清零*/
  memset(&(their_addr.sin_zero), 0, 8);

  if (connect(sockfd_, (struct sockaddr*)&their_addr,
              sizeof(struct sockaddr)) == -1) {
    /* 如果connect()建立连接错误，则显示出错误信息，退出 */
    ROS_ERROR_STREAM(server_ip << ": " << port);
    exit(1);
  }
  //设置网络通信为非阻塞
  fcntl(sockfd_, F_SETFL, O_NONBLOCK);
}
void TCPClient::loop() {
  memset(&cmd_, 0, cmd_size);
  if (recv(sockfd_, &cmd_, cmd_size, 0) == cmd_size) {
    if (cmd_.uchHead == 0xf3 && cmd_.uchEnd == 0xf4) {
      agv_robot::agv_cmd cmd_send;
      switch (cmd_.uchNaviCmd) {
        case 0x01:
          cmd_send.command = 1;
          std::cout << "cmd 1" << std::endl;
          break;
        case 0x02:
          cmd_send.command = 2;
          std::cout << "cmd 2" << std::endl;
          break;
        case 0x03:
          cmd_send.command = 3;
          std::cout << "cmd 3" << std::endl;
          float x, y, theta;
          uchar2float(&cmd_.uchNaviTarget[0], x);
          uchar2float(&cmd_.uchNaviTarget[4], y);
          uchar2float(&cmd_.uchNaviTarget[8], theta);
          cmd_send.pose.x = x;
          cmd_send.pose.y = y;
          cmd_send.pose.theta = theta;
          break;
        default:
          break;
      };
      cmd_pub.publish(cmd_send);
    }
  }
  getRobotStatus();
  if (send(sockfd_, &navi_status, status_size, 0) == -1) {
    // 如果错误，则给出错误提示，然后关闭这个新连接，退出
    perror("send err");
    close(sockfd_);
    exit(0);
  }
}
//
void TCPClient::float2uchar(float fpointer, unsigned char* a) {
  char* pchar = (char*)&fpointer;
  a[0] = *pchar;
  a[1] = *(pchar + 1);
  a[2] = *(pchar + 2);
  a[3] = *(pchar + 3);
}
void TCPClient::uchar2float(unsigned char* a, float& fpointer) {
  memcpy(&fpointer, a, 4);
}
unsigned short TCPClient::getCRC16(const unsigned char* ptr,
                                   unsigned char len) {
  unsigned char i;
  unsigned short crc = 0xFFFF;
  if (len == 0) {
    len = 1;
  }
  while (len--) {
    crc ^= *ptr;
    for (i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
    ptr++;
  }
  return (crc);
}
void TCPClient::status_callback(const agv_robot::agv_cmd::ConstPtr& msg) {
  switch (msg->nav_status) {
    case 1:
      STATUS = NAV;
      err_code = 1;
      break;
    case 2:
      STATUS = IDLE;
      err_code = 2;
      break;
    case 3:
      STATUS = IDLE;
      err_code = 3;
      break;
    case 4:
      STATUS = PAUSE;
      err_code = 4;
      break;
    case 5:
      STATUS = PAUSE;
      err_code = 5;
      break;
    case 6:
      STATUS = IDLE;
      err_code = 6;
      break;
    case 7:
      STATUS = NAV;
      err_code = 7;
      break;
    case 8:
      STATUS = IDLE;
      err_code = 8;
      break;
    case 9:
      STATUS = PAUSE;
      err_code = 9;
      break;
    default:
      break;
  }
  memset(pose, 0, sizeof(pose));
  float x = msg->pose.x;
  float y = msg->pose.y;
  float theta = msg->pose.theta;
  std::cout<<"wangjianbo  "<<x<<"     "<<y<<"        "<<theta<<std::endl;
  if (msg->pose.x == msg->pose.x || msg->pose.y == msg->pose.y ||
      msg->pose.theta == msg->pose.theta) {
    float2uchar(msg->pose.x, &pose[0]);
    float2uchar(msg->pose.y, &pose[4]);
    float2uchar(msg->pose.theta, &pose[8]);
    poseValid = true;
  } else {
    poseValid = false;
  }
}
void TCPClient::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  cmd_vel_watch = ros::Time::now();
  if (joy_vel_received) return;
  memset(velocity_, 0, sizeof(velocity_));
  float2uchar(msg->linear.x, &velocity_[0]);
  float2uchar(msg->linear.y, &velocity_[4]);
  float2uchar(msg->angular.z, &velocity_[8]);
  cmd_vel_received =
      msg->linear.x != 0 || msg->linear.y != 0 || msg->angular.z != 0;
}
void TCPClient::joy_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (cmd_vel_received && ros::Time::now() - cmd_vel_watch > ros::Duration(0.5))
    cmd_vel_received = false;
  memset(velocity_, 0, sizeof(velocity_));
  joy_vel_received =
      msg->linear.x != 0 || msg->linear.y != 0 || msg->angular.z != 0;
  if (!cmd_vel_received || joy_vel_received) {
    float2uchar(msg->linear.x, &velocity_[0]);
    float2uchar(msg->linear.y, &velocity_[4]);
    float2uchar(msg->angular.z, &velocity_[8]);
  }
}
void TCPClient::getRobotStatus() {
  memset(&navi_status, 0, status_size);
  bool isInitialized;
  navi_status.uchHead = 0xf1;
  /// bit0 bit1
  if (nh_.getParam("isInitialized", isInitialized)) {
    if (isInitialized)
      navi_status.NaviStatus[0] |= 0x02;  // 00000010
    else
      navi_status.NaviStatus[0] |= 0x01;  // 00000001
  } else
    navi_status.NaviStatus[0] |= 0x00;  // 00000000

  /// bit2 bit3
  if (access(map_name.c_str(), F_OK) == 0) {
    navi_status.NaviStatus[0] |= 0x08;
    isMapExist = true;
  } else
    navi_status.NaviStatus[0] |= 0x00;

  /// bit4 bit5
  if (STATUS == IDLE) {
    navi_status.NaviStatus[0] |= 0x00;
  } else if (STATUS == NAV) {
    navi_status.NaviStatus[0] |= 0x10;
  } else if (STATUS == PAUSE) {
    navi_status.NaviStatus[0] |= 0x20;
  }
  /// bit6 bit7
  if (poseValid)
    navi_status.NaviStatus[0] |= 0x80;
  else
    navi_status.NaviStatus[0] |= 0x00;
  navi_status.NaviStatus[1] = err_code;
  err_code = 0;
  /// 0-3 vx, 4-7 vy, 8-11 vth
  for (int i = 0; i < 12; ++i) {
    navi_status.uchAgvCmd[i] = velocity_[i];
  }
  /// x y z r p y
  for (int i = 0; i < 12; ++i) {
    navi_status.uchAGVStatus[i] = pose[i];
  }
  navi_status.uchEnd = 0xf2;
}
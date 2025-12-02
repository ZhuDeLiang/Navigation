#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "util.h"
class BaseControl {
 public:
  BaseControl(ros::NodeHandle nh, int baudrate, std::string port);
  ros::NodeHandle nh_;
  serial::Serial serial_;
  uint8_t serSendBuff_;
  ros::Time cmd_vel_watch;
  DataWithMutex<bool> cmd_vel_received = false;
  DataWithMutex<bool> joy_vel_received = false;
  bool binitdone{false};
  float abcd[4]{0, 0, 0, 0};
  class AgvCmd {
   public:
    unsigned char id = 0;
    unsigned char cmd = 0;
    unsigned char data[4];
    unsigned char crc[2];
  };
  void run();

  bool openSerialPort(int baudrate, std::string port);
  void clearError();
  void enableAGV();
  bool writeSp(AgvCmd buf, int size);
  bool sendAgvCmd(int id, unsigned char cmd, AgvCmd &buf, float value = 0.0);
  unsigned short getCRC16(const unsigned char *ptr, unsigned char len);
  void ConvertDexToIEE754(float fpointer,
                          unsigned char *a);  //十进制转化为 IEEE745 小数
  void joy_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);

  AgvCmd ac;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber joy_vel_sub;
};
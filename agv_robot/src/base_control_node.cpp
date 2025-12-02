#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "base_control.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "base_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  // ROS node initialization
  int baudrate;
  std::string port;
  nh_.param("baudrate", baudrate, (int)115200);
  nh_.param("port", port, (std::string) "/dev/ttyS0");
  BaseControl bc(nh, baudrate, port);
  // serial port subscribe something from PC
  ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
  // Pub something from serialport and i can subscribe this toptic"read"from
  // another listen_node
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    bc.run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

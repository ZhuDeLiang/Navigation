#include "tcp_client.h"

using namespace std;
void Stop(int sig);
int main(int argc, char* argv[]) {
  signal(SIGINT, Stop);
  ros::init(argc, argv, "tcp_client_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  double cycle_ms;
  nh_local.param("cycle_ms", cycle_ms, (double)200);
  ROS_INFO_STREAM("TCP Client initializing...");
  ROS_INFO_STREAM("Message send interval: "<<cycle_ms<<" ms");

  /* 套接字描述符 */
  // int numbytes;
  // char buf[MAXDATASIZE];
  TCPClient client(nh);
  ros::Rate loop_rate(1000.0 / cycle_ms);
  while (ros::ok()) {
    client.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  close(TCPClient::getSockfd());
  return 0;
}

void Stop(int sig) {
  printf("now close and quit.\n");
  if (TCPClient::getSockfd() > 0) close(TCPClient::getSockfd());
  _exit(0);
}
/*
 * Copyright (C) 2013, Osnabrück University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 14.11.2013
 *
 *      Author:
 *         Martin Günther <mguenthe@uos.de>
 *
 */


#include <sick_tim/sick_tim_common_usb.h>
#include <sick_tim/sick_tim_common_tcp.h>
#include <sick_tim/sick_tim_common_mockup.h>
#include <sick_tim/sick_tim551_2050001_parser.h>
/*
 * add by fang on 2019.3.20
 *for starting up and shutting down the laser and the motor
 */
#include <sick_tim/abstract_parser.h>
#include <sick_tim/sick_tim_common.h>
#include <std_msgs/Int8.h>

void switch_callback(  const std_msgs::Int8 &cmd ); 
int scan_run_state; //Run state of the lidar. 1(default)-on; 2-off
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim551_2050001");
  ros::NodeHandle nhPriv("~");
  ros::Subscriber power_cmd = nhPriv.subscribe ( "scan_switch", 1, switch_callback ); // Topic name: /sick_tim551_2050001/scan_switch   Message type: std_msgs/Int8   0-defalut; 1-switch on; 2-switch off.
  // check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::string hostname;
  if(nhPriv.getParam("hostname", hostname)) {
      useTCP = true;
  }
  std::string port;
  nhPriv.param<std::string>("port", port, "2112");

  int timelimit;
  nhPriv.param("timelimit", timelimit, 5);

  bool subscribe_datagram;
  int device_number;
  nhPriv.param("subscribe_datagram", subscribe_datagram, false);
  nhPriv.param("device_number", device_number, 0);

  sick_tim::SickTim5512050001Parser* parser = new sick_tim::SickTim5512050001Parser();

  double param;
  if (nhPriv.getParam("range_min", param))
  {
    parser->set_range_min(param);
  }
  if (nhPriv.getParam("range_max", param))
  {
    parser->set_range_max(param);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    parser->set_time_increment(param);
  }

  sick_tim::SickTimCommon* s = NULL;
  int result = sick_tim::ExitError;
  while (ros::ok())
  {
    // Atempt to connect/reconnect
//    if (subscribe_datagram)
//      s = new sick_tim::SickTimCommonMockup(parser);
//    else if (useTCP)
    if(useTCP)
      s = new sick_tim::SickTimCommonTcp(hostname, port, timelimit, parser);
//    else
//      s = new sick_tim::SickTimCommonUsb(parser, device_number);
    result = s->init();
    scan_run_state = 1;
    while(ros::ok() && (result == sick_tim::ExitSuccess)) //Main loop of the Sick_tim node
    {
      while(ros::ok() && (result == sick_tim::ExitSuccess) && (scan_run_state == 1)) // Loop when lidar is on
      {
	ros::spinOnce();
	result = s->loopOnce();
//	ROS_INFO("recieving data");
	if(scan_run_state == 0)
	{
	  ROS_INFO("Shutting down the laser and the motor !!!!!!!!!!!\n");
	  s->shutdownScanner();
	}
      }
      while(ros::ok() && (result == sick_tim::ExitSuccess) && (scan_run_state == 0)) // Loop when lidar is off, wait for start up command.
      {
	ros::spinOnce();
	ROS_INFO("Lidar is off, waiting for startup command.");
	ros::Duration(3.0).sleep();
	if(scan_run_state == 1)
	{
	  ROS_INFO("Starting up the laser and the motor !!!!!!!!!!\n");
	  s->startupScanner();
	}
      }
//      ros::spinOnce();
//      result = s->loopOnce();
    }

    delete s;

    if (result == sick_tim::ExitFatal)
      return result;

    if (ros::ok() && !subscribe_datagram && !useTCP)
      ros::Duration(1.0).sleep(); // Only attempt USB connections once per second
  }

  delete parser;
  return result;
}
/*
 * add by fang on 2019.3.20
 *for starting up and shutting down the lazer and the motor
 */
void switch_callback ( const std_msgs::Int8 &cmd)
    {
      int switch_flag =cmd.data;
      if( switch_flag == 1)
      {
	scan_run_state = 1;
      }
      else if( switch_flag ==2)
      {
	scan_run_state = 0;
      }
      switch_flag = 0;
    }

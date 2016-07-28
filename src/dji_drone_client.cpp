#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

#include "DroneClient.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_drone_client");
  ROS_INFO_STREAM("Initialising DJI drone client");
  ros::NodeHandle nh;
  
  DroneClient djiClient(nh) ;
  
	ros::spin();
	
	return 0 ;
}

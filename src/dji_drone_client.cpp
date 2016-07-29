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
  
  DJIDrone* drone = new DJIDrone(nh);
  DroneClient djiClient(nh) ;
  ros::Rate loop_rate(10);
  while(drone->request_sdk_permission_control()){
    loop_rate.sleep();
  }
  while (ros::ok())
  {
    djiClient.sendWaypointRequest(drone);

    ros::spinOnce();
    loop_rate.sleep();
  }
  drone->release_sdk_permission_control();
  return 0;
  
	ros::spin();
	
	return 0 ;
}

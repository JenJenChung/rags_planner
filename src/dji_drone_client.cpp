#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;


void waypointCallback(const dji_sdk::Waypoint& msg, DJIDrone * drone){
  dji_sdk::WaypointList newWPList ;
  newWPList.waypoint_list.push_back(msg) ;
  
  drone->waypoint_navigation_send_request(newWaypointList);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_drone_client");
  ROS_INFO("Initialising DJI drone client");
  ros::NodeHandle nh;
  DJIDrone* drone = new DJIDrone(nh);
  
  ros::Subscriber sub = nh.subscribe("cmd_wp", 10, &waypointCallback) ;
  
	ros::spin();
}

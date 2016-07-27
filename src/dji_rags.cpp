#include <ros/ros.h>
#include <ros/console.h>
#include "RAGS.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_rags");
  ROS_INFO_STREAM("Initialising RAGS");
  
  ros::NodeHandle nh ;
  
  RAGS RAGSDrone(nh) ;
  
  ros::spin();
  return 0;
}

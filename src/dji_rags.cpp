#include <ros/ros.h>
#include "RAGS.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_rags");
  ROS_INFO("Initialising RAGS");
  
  ros::NodeHandle nh ;
  
  RAGS RAGSDrone(nh) ;
  
  ros::spin();
  return 0;
}

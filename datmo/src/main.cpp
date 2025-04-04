#include "ros/ros.h"
#include "datmo.hpp"
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "datmo_node");

  Datmo  datmo_object;

  ros::spin();

  return 0;
}

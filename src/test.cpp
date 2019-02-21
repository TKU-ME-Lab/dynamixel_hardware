#include <ros/ros.h>

#include "dynamixel_hardware.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  std::vector<std::string> motor_names;
  for (int index=0; index < argc-1; ++index)
  {
    motor_names.push_back(argv[index+1]);
  }

  CDynamixelHardware dynamixelhardware(nh, pnh, motor_names);

  if (!dynamixelhardware.init())
  {
    ROS_FATAL("Init Failed");
    return 0;
  }

  ROS_INFO("Init done");
  
  return 0;
}
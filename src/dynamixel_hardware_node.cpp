#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

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

  BOOST_FOREACH(const std::string&name, motor_names)
  {
    ROS_INFO_STREAM("Name:" + name);
  }

  CDynamixelHardware dxl_hardware(nh, pnh, motor_names);
  controller_manager::ControllerManager CM(&dxl_hardware, nh);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (!dxl_hardware.init())
  {
    ROS_FATAL("Init Failed");
    return 0;
  }

  ros::Rate contoroller_rate(1000);
  ros::Time last = ros::Time::now();
  while(ros::ok())
  {
    dxl_hardware.read();
    ros::Time now = ros::Time::now();
    CM.update(now, now-last);
    dxl_hardware.write();
    last = now;
    contoroller_rate.sleep();
  }
  
  return 0;
}
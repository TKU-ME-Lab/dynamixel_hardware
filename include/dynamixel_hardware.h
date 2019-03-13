#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

typedef enum{
  POSITION_CONTROL_MODE, VELOCITY_CONTROL_MODE
}OperationMode;

typedef struct{
  uint32_t id;
  uint8_t torque_enable;
  uint8_t moving;
  double present_position;
  double present_velocity;
  double present_current;

  double goal_position;
  double goal_velocity;
  //double goal_current;
}DynamixelInfoList;

class CDynamixelHardware: public hardware_interface::RobotHW
{
private:
  typedef std::map<std::string, DynamixelInfoList> DynamixelInfoMap;
  typedef std::map<std::string, const ControlItem*> ControlItemMap;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;
  
  hardware_interface::ActuatorStateInterface m_asi;
  hardware_interface::PositionActuatorInterface m_api;
  hardware_interface::VelocityActuatorInterface m_avi;
  //hardware_interface::EffortActuatorInterface m_aei;
  
  DynamixelWorkbench* m_pDxl_wb;
  DynamixelInfoMap m_DxlMap;
  ControlItemMap m_control_items;

  uint8_t* m_dxl_id_array;
  uint8_t m_dxl_count;

  transmission_interface::RobotTransmissions m_robot_transmissions;

  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> m_transmission_loader;

  OperationMode m_OperationMode;
  bool m_has_init;
  bool m_valid;
  
public:
  CDynamixelHardware(ros::NodeHandle&, ros::NodeHandle&, const std::vector<std::string>);

  bool init();
  void read();
  void write();

};
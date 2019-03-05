#include "dynamixel_hardware.h"
#include <yaml-cpp/yaml.h>
#include <boost/foreach.hpp>

CDynamixelHardware::CDynamixelHardware(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> motor_names):
m_nh(nh), m_private_nh(pnh), m_has_init(false), m_valid(false)
{
  m_pDxl_wb = new DynamixelWorkbench;

  if (!nh.hasParam("DynamixelConfigs"))
  {
    ROS_WARN("Didn't have DynamixelConfigs parameter");
    return;
  }

  ros::NodeHandle Config_nh(m_private_nh, "DynamixelConfigs");

  BOOST_FOREACH(const std::string&motor_name, motor_names)
  {
    if (nh.hasParam(motor_name))
    {
      ros::NodeHandle motor_nh(Config_nh, motor_name);
      int id;
      if (motor_nh.getParam("ID", id))
      {
        DynamixelInfoList infolist;
        infolist.id = (uint32_t)id;
        m_DxlMap[motor_name] = infolist;
      }
      else
      {
        ROS_WARN_STREAM("Motor:" + motor_name + " didn't have ID");
      }
    }
    else
    {
      ROS_WARN_STREAM("Not found " + motor_name + "in DynamixelConfigs");
      continue;
    }
  }

  ROS_INFO_STREAM("Find " + std::to_string(m_DxlMap.size()) + "ID count");

  std::string port;
  int baud_rate;
  if (!Config_nh.getParam("port", port))
  {
    ROS_ERROR("Didn't have port parameter");
    return;
  }

  if (!Config_nh.getParam("baud_rate", baud_rate))
  {
    ROS_ERROR("Didn't have baud_rate parameter");
    return;
  }
  const char *log;

  if (!m_pDxl_wb->begin(port.c_str(), baud_rate, &log))
  {
    ROS_ERROR_STREAM("Port:" + port + ", Baud Rate:" + std::to_string(baud_rate) + ", Begin Failed");
  }

  //ros::NodeHandle dynamixel_config_nh(m_private_nh, "DynamixelConfigs");
  std::string operation_mode;
  if (Config_nh.getParam("operation_mode", operation_mode))
  {
    if (operation_mode == "PositionMode")
    {
      m_OperationMode = POSITION_CONTROL_MODE;
    }
    else if (operation_mode == "VelocityMode")
    {
      m_OperationMode = VELOCITY_CONTROL_MODE;
    }
    else
    {
      ROS_WARN("Didn't found OperationMode in ROS Parameter, so OperationMode set Position Mode");
      m_OperationMode = POSITION_CONTROL_MODE;
    }
  }

  m_valid = true;
}

bool CDynamixelHardware::init()
{
  if (m_valid == false)
  {
    ROS_WARN("Dynamixelworkbench invalid");
    return false;
  } 

  const char* log;

  for (DynamixelInfoMap::iterator iter = m_DxlMap.begin(); iter != m_DxlMap.end(); iter++)
  {
    uint16_t model_number = 0;
    if (m_pDxl_wb->ping((uint8_t)iter->second.id, &model_number, &log))
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", iter->second.id);
      return false;
    }
    else
    {
      ROS_INFO("Name: %s, ID: %d, Model Number: %d", iter->first.c_str(), iter->second.id, model_number);

      if (m_OperationMode == POSITION_CONTROL_MODE)
      {
        if (!m_pDxl_wb->setPositionControlMode((uint8_t)iter->second.id, &log))
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("ID: '%d', Can't Set Position Mode", iter->second.id);
          return false;
        }
      }
      else if (m_OperationMode == VELOCITY_CONTROL_MODE)
      {
        if (!m_pDxl_wb->setVelocityControlMode((uint8_t)iter->second.id, &log))
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("ID: '%d', Can't Set Velocity Mode", iter->second.id);
          return false;
        }
      }
    }
    
  }
  
  auto it = m_DxlMap.begin();

  const ControlItem *goal_position = m_pDxl_wb->getItemInfo(it->second.id, "Goal_Position");
  if (goal_position == NULL) return false;
  const ControlItem *goal_velocity = m_pDxl_wb->getItemInfo(it->second.id, "Goal_Velocity");
  if (goal_velocity == NULL) return false;

  m_control_items["Goal_Position"] = goal_position;
  m_control_items["Goal_Velocity"] = goal_velocity;

  for (DynamixelInfoMap::iterator it = m_DxlMap.begin(); it != m_DxlMap.end(); it++)
  {
    hardware_interface::ActuatorStateHandle statehandle(it->first, &it->second.present_position, &it->second.present_velocity, 
                                                      &it->second.present_current);
    ROS_INFO_STREAM("Create ActuatorStateHandle, Name: " + statehandle.getName());
    m_asi.registerHandle(statehandle);
    hardware_interface::ActuatorHandle position_handle(statehandle, &it->second.goal_position);
    m_api.registerHandle(position_handle);
    hardware_interface::ActuatorHandle velocity_handle(statehandle, &it->second.goal_velocity);
    m_avi.registerHandle(velocity_handle);
    //hardware_interface::ActuatorHandle current_handle(statehandle, &dynamixel->goal_current);
    //m_aei.registerHandle(current_handle)
  }

  registerInterface(&m_asi);
  registerInterface(&m_api);
  registerInterface(&m_avi);

  return true;
}

void CDynamixelHardware::read()
{
  const char* log;
  for (std::map<std::string, DynamixelInfoList>::iterator it = m_DxlMap.begin(); it != m_DxlMap.end(); it++)
  {
    int32_t torque_enable, moving, position, velocity, current;
    m_pDxl_wb->itemRead((uint8_t)it->second.id, "Torque_Enable", &torque_enable, &log);
    m_pDxl_wb->itemRead((uint8_t)it->second.id, "Moving", &moving, &log);
    m_pDxl_wb->itemRead((uint8_t)it->second.id, "Present_Position", &position, &log);
    m_pDxl_wb->itemRead((uint8_t)it->second.id, "Present_Velocity", &velocity, &log);
    m_pDxl_wb->itemRead((uint8_t)it->second.id, "Present_Current", &current, &log);

    it->second.torque_enable = (uint8_t)torque_enable;
    it->second.moving = (uint8_t)moving;
    it->second.present_position = (double)m_pDxl_wb->convertValue2Radian((uint8_t)it->second.id, position);
    it->second.torque_enable = (double)m_pDxl_wb->convertValue2Velocity((uint8_t)it->second.id, velocity);
    it->second.torque_enable = (double)m_pDxl_wb->convertValue2Current((int16_t)current);
  }
  
}

void CDynamixelHardware::write()
{

}
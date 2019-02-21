#include "dynamixel_hardware.h"
#include <yaml-cpp/yaml.h>
#include <boost/foreach.hpp>

CDynamixelHardware::CDynamixelHardware(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> motor_names):
m_nh(nh), m_private_nh(pnh), m_has_init(false), m_valid(false)
{
  m_pDxl_wb = new DynamixelWorkbench;

  std::string yamlfile_name;
  if (!nh.getParam("dynamixel_info", yamlfile_name))
  {
    ROS_WARN("Didn't have dynamixel_info parameter");
    return;
  }

  YAML::Node yaml_node;
  yaml_node = YAML::LoadFile(yamlfile_name.c_str());

  if (yaml_node == NULL)
  {
    ROS_WARN_STREAM("Didn't have " + yamlfile_name + " this file");
    return;
  }

  for (auto const& motor_name:motor_names)
  {
    YAML::Node joint_item = yaml_node[motor_name];
    for (YAML::const_iterator it_item = joint_item.begin(); it_item != joint_item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      uint32_t id = it_item->second.as<uint32_t>();
      DynamixelInfoList infolist;
      infolist.id = id;

      if (item_name == "ID")
      {
        m_DxlMap[item_name] = infolist;
      }
    }
  }

  std::string port;
  int baud_rate;
  if (!m_nh.getParam("port", port))
  {
    ROS_ERROR("Didn't have port parameter");
    return;
  }

  if (!m_nh.getParam("baud_rate", baud_rate))
  {
    ROS_ERROR("Didn't have baud_rate parameter");
    return;
  }

  ros::NodeHandle dynamixel_config_nh(m_private_nh, "DynamixelConfigs");
  std::string operation_mode;
  if (dynamixel_config_nh.getParam("operation_mode", operation_mode))
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
  if (m_valid == false) return false;

  const char* log;

  for (auto const& dxl:m_DxlMap)
  {
    uint16_t model_number = 0;
    if (m_pDxl_wb->ping((uint8_t)dxl.second.id, &model_number, &log))
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second.id);
      return false;
    }
    else
    {
      ROS_INFO("Name: %s, ID: %d, Model Number: %d", dxl.first.c_str(), dxl.second.id, model_number);
    }
  }

  for (auto const& dxl:m_DxlMap)
  {
    if (m_OperationMode == POSITION_CONTROL_MODE)
    {
      if (!m_pDxl_wb->setPositionControlMode((uint8_t)dxl.second.id, &log))
      {
        ROS_ERROR("%s", log);
        ROS_ERROR("ID: '%d', Can't Set Position Mode", dxl.second.id);
        return false;
      }
    }
    else if (m_OperationMode == VELOCITY_CONTROL_MODE)
    {
      if (!m_pDxl_wb->setVelocityControlMode((uint8_t)dxl.second.id, &log))
      {
        ROS_ERROR("%s", log);
        ROS_ERROR("ID: '%d', Can't Set Velocity Mode", dxl.second.id);
        return false;
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

  for (std::map<std::string, DynamixelInfoList>::iterator it = m_DxlMap.begin(); it != m_DxlMap.end(); it++)
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
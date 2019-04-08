#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_hardware_msgs/DynamixelControlTable.h>
#include <dynamixel_hardware_msgs/GetDynamixelControlTable.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

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

class CDynamixelTool
{
private:
  typedef std::map<std::string, DynamixelInfoList> DynamixelInfoMap;
  typedef std::map<std::string, const ControlItem*> ControlItemMap;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  DynamixelWorkbench* m_pDxl_wb;
  DynamixelInfoMap m_DxlMap;
  ControlItemMap m_control_items;

  uint8_t* m_dxl_id_array;
  uint8_t m_dxl_count;

  OperationMode m_OperationMode;
  bool m_has_init;
  bool m_valid;

  ros::ServiceServer m_ServiceServer_GetControlTable;
  ros::ServiceServer m_ServiceServer_Command;

  bool Service_GetControlTable_Callback(dynamixel_hardware_msgs::GetDynamixelControlTable::Request&,
                                        dynamixel_hardware_msgs::GetDynamixelControlTable::Response&);

  bool Service_Command_Callback(dynamixel_workbench_msgs::DynamixelCommand::Request&,
                                dynamixel_workbench_msgs::DynamixelCommand::Response&);

public:
  CDynamixelTool(ros::NodeHandle&, ros::NodeHandle&, const std::vector<std::string>);
  ~CDynamixelTool();

  bool init();
  void read();
  void write();

};

bool CDynamixelTool::Service_GetControlTable_Callback(dynamixel_hardware_msgs::GetDynamixelControlTable::Request&req, 
                                                      dynamixel_hardware_msgs::GetDynamixelControlTable::Response&res)
{
  uint result = 1;
  dynamixel_hardware_msgs::DynamixelControlTable control_table;
  int32_t id(0), operate_mode(0), homing_offset(0), torque_enable(0), velocity_p_gain(0), velocity_i_gain(0), position_p_gain(0);
  result = m_pDxl_wb->itemRead(req.id, "ID", &id);
  result = m_pDxl_wb->itemRead(req.id, "Operating_Mode", &operate_mode);
  result = m_pDxl_wb->itemRead(req.id, "Homing_Offset", &homing_offset);
  result = m_pDxl_wb->itemRead(req.id, "Torque_Enable", &torque_enable);
  result = m_pDxl_wb->itemRead(req.id, "Velocity_P_Gain", &velocity_p_gain);
  result = m_pDxl_wb->itemRead(req.id, "Velocity_I_Gain", &velocity_i_gain);
  result = m_pDxl_wb->itemRead(req.id, "Position_P_Gain", &position_p_gain);

  control_table.id              = id;
  control_table.operating_mode  = operate_mode;
  control_table.homing_offset   = homing_offset;
  control_table.torque_enable   = torque_enable;
  control_table.velocity_p_gain = velocity_p_gain;
  control_table.velocity_i_gain = velocity_i_gain;
  control_table.position_p_gain = position_p_gain;

  res.control_table = control_table;

  if (result == false)
  {
    return false;
  }

  return true;
}

bool CDynamixelTool::Service_Command_Callback(dynamixel_workbench_msgs::DynamixelCommand::Request& req,
                                              dynamixel_workbench_msgs::DynamixelCommand::Response& res)
{
  bool result = false;
  if (req.command == "Set")
  {
    result = m_pDxl_wb->itemWrite(req.id, req.addr_name.c_str(), req.value);
  }
}

CDynamixelTool::CDynamixelTool(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> motor_names):
m_nh(nh), m_private_nh(pnh), m_has_init(false), m_valid(false)
{
  m_pDxl_wb = new DynamixelWorkbench;

  ros::NodeHandle Config_nh(m_private_nh, "DynamixelConfigs");

  m_dxl_id_array = new uint8_t[motor_names.size()];
  m_dxl_count = motor_names.size();
  uint8_t array_index = 0;

  BOOST_FOREACH(const std::string&motor_name, motor_names)
  {
    ros::NodeHandle motor_nh(Config_nh, motor_name);
    int id;
    if (motor_nh.getParam("ID", id))
    {
      DynamixelInfoList infolist;
      infolist.id = (uint32_t)id;
      m_DxlMap[motor_name] = infolist;
      m_dxl_id_array[array_index] = (uint8_t)id;
      array_index++;
    }
    else
    {
      ROS_WARN_STREAM("Motor:" + motor_name + " didn't have ID");
      continue;
    }
  }

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

  m_OperationMode = POSITION_CONTROL_MODE;

  m_ServiceServer_GetControlTable = m_nh.advertiseService(ros::this_node::getName()+"/GetControlTable", &CDynamixelTool::Service_GetControlTable_Callback, this);
  m_ServiceServer_Command         = m_nh.advertiseService(ros::this_node::getName()+"/Command", &CDynamixelTool::Service_Command_Callback, this);

  m_valid = true;
}

CDynamixelTool::~CDynamixelTool()
{
  for (DynamixelInfoMap::iterator iter = m_DxlMap.begin(); iter != m_DxlMap.end(); iter++)
  {
    uint16_t model_number = 0;
    m_pDxl_wb->torqueOff((uint8_t)iter->second.id);
  } 
}

bool CDynamixelTool::init()
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

    if (!m_pDxl_wb->ping((uint8_t)iter->second.id, &model_number, &log))
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID:%d", iter->second.id);
      return false;
    }
    else
    {
      ROS_INFO("Name: %s, ID: %d, Model Number: %d", iter->first.c_str(), iter->second.id, model_number);
      m_pDxl_wb->torqueOff((uint8_t)iter->second.id);
    }
  }  

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_test");
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

  CDynamixelTool dynamixel_tool(nh, pnh, motor_names);

  ros::Rate rate(1000);

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
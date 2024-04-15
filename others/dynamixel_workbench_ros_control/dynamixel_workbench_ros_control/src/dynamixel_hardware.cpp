/*******************************************************************************
 * Copyright 2018 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Taehun Lim (Darby) */
/* Editors: Yoshimaru Tanaka */

#include "dynamixel_workbench_ros_control/dynamixel_hardware.h"
// #include <boost/assign/list_of.hpp>

namespace dynamixel_workbench_ros_control
{
DynamixelHardware::DynamixelHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
  : nh_(nh), private_nh_(private_nh), joints_(0), is_first_(true)
// is_moving_(false) // necessary?
{
  initializeDynamixelHardware();
  registerControlInterfaces();
}

DynamixelHardware::~DynamixelHardware()
{
}

bool DynamixelHardware::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  ROS_INFO_STREAM("Init port " << port_name << ", baud_rate " << baud_rate);

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelHardware::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();

    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    DynamixelYamlConfig dynamixel_yaml_config;
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();

      if (item_name == "ID")
      {
        int32_t value = it_item->second.as<int32_t>();

        dynamixel_[name] = value;
        dynamixel_yaml_config.id = value;

        ItemValue item_value = { item_name, value };
        std::pair<std::string, ItemValue> info(name, item_value);

        dynamixel_info_.push_back(info);
      }
      else if (item_name == "control_mode")
      {
        dynamixel_yaml_config.control_mode = it_item->second.as<std::string>();
      }
      else if (item_name == "spin")
      {
        dynamixel_yaml_config.spin = it_item->second.as<double>();
        if (dynamixel_yaml_config.spin > 0)
          dynamixel_yaml_config.spin = 1.0;
        else
          dynamixel_yaml_config.spin = -1.0;
      }
    }
    // ROS_INFO_STREAM("getDynamixelsInfo: joint " << name << ". ID = " << dynamixel_yaml_config.id
    //                                            << ", control_mode = " << dynamixel_yaml_config.control_mode);

    Joint joint;
    joint.dynamixel_config = dynamixel_yaml_config;
    joint.name = name;
    joints_.push_back(joint);
  }

  return true;
}

bool DynamixelHardware::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl : dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelHardware::initDynamixels(void)
{
  const char* log;
  bool result = false;

  for (auto iter = joints_.begin(); iter != joints_.end(); iter++)
  {
    dxl_wb_->torqueOff((uint8_t)iter->dynamixel_config.id);

    if (iter->dynamixel_config.control_mode == "position")
    {
      result = dxl_wb_->setPositionControlMode((uint8_t)iter->dynamixel_config.id, &log);
      if (result == false)
      {
        ROS_ERROR("Error setting position control mode in servo %d: %s", iter->dynamixel_config.id, log);
      }
      else
      {
        ROS_INFO_STREAM("Setting position control mode for servo " << iter->dynamixel_config.id);
      }
    }
    else if (iter->dynamixel_config.control_mode == "velocity")
    {
      result = dxl_wb_->setVelocityControlMode((uint8_t)iter->dynamixel_config.id, &log);
      if (result == false)
      {
        ROS_ERROR("Error setting velocity control mode in servo %d: %s", iter->dynamixel_config.id, log);
      }
      else
      {
        ROS_INFO_STREAM("Setting velocity control mode for servo " << iter->dynamixel_config.id);
      }
    }

    dxl_wb_->torqueOn((uint8_t)iter->dynamixel_config.id);
  }

  return true;
}

bool DynamixelHardware::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem* goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL)
    return false;

  const ControlItem* goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)
    goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)
    return false;

  const ControlItem* present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL)
    return false;

  const ControlItem* present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)
    present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL)
    return false;

  const ControlItem* present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)
    present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL)
    return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool DynamixelHardware::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address,
                                        control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address,
                                        control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address =
        std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as
     * below.*/
    // uint16_t read_length = control_items_["Present_Position"]->data_length +
    // control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length +
                           control_items_["Present_Velocity"]->data_length +
                           control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(start_address, read_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

bool DynamixelHardware::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;

  int32_t get_position[dxl_name.size()];

  uint8_t id_array[dxl_name.size()];
  uint8_t id_cnt = 0;

  for (auto const& name : dxl_name)
    id_array[id_cnt++] = dynamixel_[name];

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result =
        dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, dxl_name.size(), &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    WayPoint wp;

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length, get_position, &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      for (uint8_t index = 0; index < id_cnt; index++)
      {
        wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
        pre_goal_.push_back(wp);
      }
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl : dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second, control_items_["Present_Position"]->address,
                                     control_items_["Present_Position"]->data_length, &read_position, &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      pre_goal_.push_back(wp);
    }
  }

  return result;
}

void DynamixelHardware::initializeDynamixelHardware()
{
  // init workbench class
  dxl_wb_ = new DynamixelWorkbench;

  std::string port_name;
  private_nh_.param<std::string>("port", port_name, "/dev/ttyUSB0");

  int baud_rate;
  private_nh_.param<int>("baud_rate", baud_rate, 57600);

  std::string yaml_file;
  private_nh_.param<std::string>("dynamixel_info", yaml_file, "");

  bool result = false;

  result = initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return;
  }

  result = getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return;
  }

  result = loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return;
  }

  result = initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return;
  }

  result = initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return;
  }

  result = initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return;
  }
}

void DynamixelHardware::registerControlInterfaces()
{
  position_joints_size_ = velocity_joints_size_ = effort_joints_size_ = 0;

  for (auto iter = joints_.begin(); iter != joints_.end(); iter++)
  {
    ROS_INFO("joint_name : %s, servo ID: %d, control_mode = %s, spin = %.0f", iter->name.c_str(),
             iter->dynamixel_config.id, iter->dynamixel_config.control_mode.c_str(), iter->dynamixel_config.spin);

    hardware_interface::JointStateHandle joint_state_handle(iter->name.c_str(), &iter->position, &iter->velocity,
                                                            &iter->effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    if (iter->dynamixel_config.control_mode == "position")
    {
      hardware_interface::JointHandle position_joint_handle(joint_state_handle, &iter->command);
      position_joint_interface_.registerHandle(position_joint_handle);
      position_joints_size_++;
    }
    else if (iter->dynamixel_config.control_mode == "velocity")
    {
      hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &iter->command);
      velocity_joint_interface_.registerHandle(velocity_joint_handle);
      velocity_joints_size_++;
    }
    /*
    // NOT AVAILABLE FOR NOW
    else if (iter->dynamixel_config.control_mode == "effort")
    {
      hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &iter->command);
      effort_joint_interface_.registerHandle(effort_joint_handle);
      effort_joints_size_++;
    }*/
    else
    {
      ROS_ERROR_STREAM("Unknown control control_mode " << iter->dynamixel_config.control_mode << " for servo "
                                                       << iter->dynamixel_config.id);
      exit(-1);
    }
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  // registerInterface(&effort_joint_interface_);
}

void DynamixelHardware::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  bool result = false;
  const char* log = NULL;

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl : dynamixel_)
    id_array[id_cnt++] = (uint8_t)dxl.second;

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result =
        dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, dynamixel_.size(), &log);
    if (result == false)
    {
      ROS_ERROR("read:syncRead: %s", log);
      return;
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items_["Present_Current"]->address,
                                      control_items_["Present_Current"]->data_length, get_current, &log);
    if (result == false)
    {
      ROS_ERROR("read:getSyncReadData:Current: %s", log);
      return;
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items_["Present_Velocity"]->address,
                                      control_items_["Present_Velocity"]->data_length, get_velocity, &log);
    if (result == false)
    {
      ROS_ERROR("read:getSyncReadData:Velocity: %s", log);
      return;
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length, get_position, &log);
    if (result == false)
    {
      ROS_ERROR("read:getSyncReadData:Position: %s", log);
      return;
    }

    for (uint8_t index = 0; index < id_cnt; index++)
    {
      // index 0, 1, 2, 3, 4
      // joints_ 1, 2, 3, 4, 5
      // id_array 1, 5, 4, 2, 3
      double spin = joints_[id_array[index] - 1].dynamixel_config.spin;
      joints_[id_array[index] - 1].position =
          dxl_wb_->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]) * spin;
      joints_[id_array[index] - 1].velocity =
          dxl_wb_->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]) * spin;
      // joints_[index].current = get_current[index];

      if (strcmp(dxl_wb_->getModelName((uint8_t)id_array[index]), "XL-320") == 0)
        joints_[id_array[index] - 1].effort = dxl_wb_->convertValue2Load((int16_t)get_current[index]) * spin;
      else
        joints_[id_array[index] - 1].effort = dxl_wb_->convertValue2Current((int16_t)get_current[index]) * spin;

      // if (is_first_ == true)
      //  joints_[id_array[index] - 1].command = joints_[id_array[index] - 1].position;
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                              control_items_["Present_Velocity"]->data_length +
                              control_items_["Present_Current"]->data_length;
    uint32_t get_all_data[length_of_data];
    uint8_t dxl_cnt = 0;
    for (auto const& dxl : dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second, control_items_["Present_Position"]->address, length_of_data,
                                     get_all_data, &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }
      double spin = joints_[(uint8_t)dxl.second - 1].dynamixel_config.spin;
      // ID required to get model name
      joints_[(uint8_t)dxl.second - 1].position =
          dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)DXL_MAKEWORD(get_all_data[0], get_all_data[1])) *
          spin;
      joints_[(uint8_t)dxl.second - 1].velocity =
          dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)DXL_MAKEWORD(get_all_data[2], get_all_data[3])) *
          spin;
      joints_[(uint8_t)dxl.second - 1].effort =
          dxl_wb_->convertValue2Load(DXL_MAKEWORD(get_all_data[4], get_all_data[5])) * spin;

      dxl_cnt++;
    }
  }
}

void DynamixelHardware::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  bool result = false;
  const char* log = NULL;

  uint8_t dynamixel_id_position_array[position_joints_size_];
  int32_t dynamixel_position_value_array[position_joints_size_];
  int32_t dynamixel_id_position_counter = 0;
  uint8_t dynamixel_id_velocity_array[position_joints_size_];
  int32_t dynamixel_velocity_value_array[position_joints_size_];
  int32_t dynamixel_id_velocity_counter = 0;

  /*for (auto const& dxl : dynamixel_)
    id_array[id_cnt++] = (uint8_t)dxl.second;

  for (uint8_t index = 0; index < id_cnt; index++)
    dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], joints_[id_array[index] - 1].command);
    */
  // split different servos by control
  for (auto iter = joints_.begin(); iter != joints_.end(); iter++)
  {
    double spin = iter->dynamixel_config.spin;

    if (iter->dynamixel_config.control_mode == "position")
    {
      dynamixel_id_position_array[dynamixel_id_position_counter] = (uint8_t)iter->dynamixel_config.id;
      dynamixel_position_value_array[dynamixel_id_position_counter] =
          dxl_wb_->convertRadian2Value((uint8_t)iter->dynamixel_config.id, iter->command * spin);
      dynamixel_id_position_counter++;
    }
    else if (iter->dynamixel_config.control_mode == "velocity")
    {
      dynamixel_id_velocity_array[dynamixel_id_velocity_counter] = (uint8_t)iter->dynamixel_config.id;
      dynamixel_velocity_value_array[dynamixel_id_velocity_counter] =
          dxl_wb_->convertVelocity2Value((uint8_t)iter->dynamixel_config.id, iter->command * spin);
      dynamixel_id_velocity_counter++;
      // ROS_INFO_STREAM("ID " << iter->dynamixel_config.id << ", command = " << iter->command);
    }
  }
  // Sync position servos
  if (position_joints_size_ > 0)
  {
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, dynamixel_id_position_array,
                                dynamixel_id_position_counter, dynamixel_position_value_array, 1, &log);
    if (result == false)
    {
      ROS_ERROR("Error writing command for position servos: %s", log);
    }
  }
  // Sync velocity servos
  if (velocity_joints_size_ > 0)
  {
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, dynamixel_id_velocity_array,
                                dynamixel_id_velocity_counter, dynamixel_velocity_value_array, 1, &log);
    if (result == false)
    {
      ROS_ERROR("Error writing command for position servos: %s", log);
    }
  }
}

}  // namespace dynamixel_workbench_ros_control

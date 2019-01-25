#include "robotis_controller/robotis_controller.h"

using namespace robotis_framework;

RobotisController::RobotisController()
    : is_timer_running_(false),
      is_offset_enabled_(true),
      stop_timer_(false),
      init_pose_loaded_(false),
      timer_thread_(0),
      controller_mode_(MotionModuleMode),
      DEBUG_PRINT(false),
      robot_(0),
      gazebo_mode_(false),
      gazebo_robot_name_("robotis")
{
    direct_sync_write_.clear();
}

void RobotisController::initializeSyncWrite()
{
  if (gazebo_mode_ == true)
    return;

  //RCLCPP_INFO(robot_node_->get_logger(),"FIRST BULKREAD");
  for (auto& it : port_to_bulk_read_)
    it.second->txRxPacket();
  for(auto& it : port_to_bulk_read_)
  {
    int error_count = 0;
    int result = COMM_SUCCESS;
    do
    {
      if (++error_count > 10)
      {
        RCLCPP_ERROR(robot_node_->get_logger(),"[RobotisController] first bulk read fail!!");
        exit(-1);
      }
      usleep(10 * 1000);
      result = it.second->txRxPacket();
    } while (result != COMM_SUCCESS);
  }
  init_pose_loaded_ = true;
  //RCLCPP_INFO(robot_node_->get_logger(),"FIRST BULKREAD END");

  // clear syncwrite param setting
  for (auto& it : port_to_sync_write_position_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_current_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }

  // set init syncwrite param(from data of bulkread)
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel *dxl = it.second;

    for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
    {
      uint32_t  read_data = 0;
      uint8_t   sync_write_data[4];

      if (port_to_bulk_read_[dxl->port_name_]->isAvailable(dxl->id_,
                                                          dxl->bulk_read_items_[i]->address_,
                                                          dxl->bulk_read_items_[i]->data_length_) == true)
      {
        read_data = port_to_bulk_read_[dxl->port_name_]->getData(dxl->id_,
                                                                dxl->bulk_read_items_[i]->address_,
                                                                dxl->bulk_read_items_[i]->data_length_);

        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

        if ((dxl->present_position_item_ != 0) &&
            (dxl->bulk_read_items_[i]->item_name_ == dxl->present_position_item_->item_name_))
        {
          if(is_offset_enabled_)
            dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(read_data) - dxl->dxl_state_->position_offset_;   // remove offset
          else
            dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(read_data);
          dxl->dxl_state_->goal_position_ = dxl->dxl_state_->present_position_;

          port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
        }
        else if ((dxl->position_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_p_gain_ = read_data;
        }
        else if ((dxl->position_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_i_gain_ = read_data;
        }
        else if ((dxl->position_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_d_gain_ = read_data;
        }
        else if ((dxl->present_velocity_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_velocity_item_->item_name_))
        {
          dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(read_data);
          dxl->dxl_state_->goal_velocity_ = dxl->dxl_state_->present_velocity_;
        }
        else if ((dxl->velocity_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_p_gain_ = read_data;
        }
        else if ((dxl->velocity_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_i_gain_ = read_data;
        }
        else if ((dxl->velocity_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_d_gain_item_->item_name_))
        {   
          dxl->dxl_state_->velocity_d_gain_ = read_data;
        }
        else if ((dxl->present_current_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_current_item_->item_name_))
        {
          dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(read_data);
          dxl->dxl_state_->goal_torque_ = dxl->dxl_state_->present_torque_;
        }
      }
    }
  }
}

bool RobotisController::initialize(const std::string robot_file_path, const std::string init_file_path)
{
//   std::string dev_desc_dir_path = ros::package::getPath("robotis_device") + "/devices";
  std::string dev_desc_dir_path;
  robot_node_->get_parameter("robotis_device", dev_desc_dir_path);
  dev_desc_dir_path = dev_desc_dir_path + "/devices";

  // load robot info : port , device
  robot_ = new Robot(robot_file_path, dev_desc_dir_path);

  if (gazebo_mode_ == true)
  {
    queue_thread_ = boost::thread(boost::bind(&RobotisController::msgQueueThread, this));
    return true;
  }

  for (auto& it : robot_->ports_)
  {
    std::string               port_name           = it.first;
    dynamixel::PortHandler   *port                = it.second;
    dynamixel::PacketHandler *default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port->setBaudRate(port->getBaudRate()) == false)
    {
      RCLCPP_ERROR(robot_node_->get_logger(), "PORT [%s] SETUP ERROR! (baudrate: %d)", port_name.c_str(), port->getBaudRate());
      exit(-1);
    }

    // get the default device info of the port
    std::string default_device_name = robot_->port_default_device_[port_name];
    auto dxl_it = robot_->dxls_.find(default_device_name);
    auto sensor_it = robot_->sensors_.find(default_device_name);
    if (dxl_it != robot_->dxls_.end())
    {
      Dynamixel *default_device = dxl_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);

      if (default_device->goal_position_item_ != 0)
      {
        port_to_sync_write_position_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_position_item_->address_,
                                            default_device->goal_position_item_->data_length_);
      }

      if (default_device->position_p_gain_item_ != 0)
      {
        port_to_sync_write_position_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_p_gain_item_->address_,
                                            default_device->position_p_gain_item_->data_length_);
      }

      if (default_device->position_i_gain_item_ != 0)
      {
        port_to_sync_write_position_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_i_gain_item_->address_,
                                            default_device->position_i_gain_item_->data_length_);
      }

      if (default_device->position_d_gain_item_ != 0)
      {
        port_to_sync_write_position_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_d_gain_item_->address_,
                                            default_device->position_d_gain_item_->data_length_);
      }

      if (default_device->goal_velocity_item_ != 0)
      {
        port_to_sync_write_velocity_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_velocity_item_->address_,
                                            default_device->goal_velocity_item_->data_length_);
      }

      if (default_device->velocity_p_gain_item_ != 0)
      {
        port_to_sync_write_velocity_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_p_gain_item_->address_,
                                            default_device->velocity_p_gain_item_->data_length_);
      }

      if (default_device->velocity_i_gain_item_ != 0)
      {
        port_to_sync_write_velocity_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_i_gain_item_->address_,
                                            default_device->velocity_i_gain_item_->data_length_);
      }

      if (default_device->velocity_d_gain_item_ != 0)
      {
        port_to_sync_write_velocity_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_d_gain_item_->address_,
                                            default_device->velocity_d_gain_item_->data_length_);
      }

      if (default_device->goal_current_item_ != 0)
      {
        port_to_sync_write_current_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_current_item_->address_,
                                            default_device->goal_current_item_->data_length_);
      }
    }
    else if (sensor_it != robot_->sensors_.end())
    {
      Sensor *_default_device = sensor_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(_default_device->protocol_version_);
    }

    port_to_bulk_read_[port_name] = new dynamixel::GroupBulkRead(port, default_pkt_handler);
  }

  // (for loop) check all dxls are connected.
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (ping(joint_name) != 0)
    {
      usleep(10 * 1000);
      if (ping(joint_name) != 0)
        RCLCPP_ERROR(robot_node_->get_logger(),"JOINT[%s] does NOT respond!!", joint_name.c_str());
    }
  }

  initializeDevice(init_file_path);

  queue_thread_ = boost::thread(boost::bind(&RobotisController::msgQueueThread, this));
  return true;
}

void RobotisController::initializeDevice(const std::string init_file_path)
{
  // device initialize
  if (DEBUG_PRINT)
    RCLCPP_WARN(robot_node_->get_logger(),"INIT FILE LOAD");

  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(init_file_path.c_str());

    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++)
    {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];
      if (joint_node.size() == 0)
        continue;

      Dynamixel *dxl = NULL;
      auto dxl_it = robot_->dxls_.find(joint_name);
      if (dxl_it != robot_->dxls_.end())
        dxl = dxl_it->second;

      if (dxl == NULL)
      {
        RCLCPP_WARN(robot_node_->get_logger(),"Joint [%s] was not found.", joint_name.c_str());
        continue;
      }
      if (DEBUG_PRINT)
        RCLCPP_INFO(robot_node_->get_logger(),"JOINT_NAME: %s", joint_name.c_str());

      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++)
      {
        std::string item_name = it_joint->first.as<std::string>();

        if (DEBUG_PRINT)
          RCLCPP_INFO(robot_node_->get_logger(),"  ITEM_NAME: %s", item_name.c_str());

        uint32_t value = it_joint->second.as<uint32_t>();

        ControlTableItem *item = dxl->ctrl_table_[item_name];
        if (item == NULL)
        {
          RCLCPP_WARN(robot_node_->get_logger(),"Control Item [%s] was not found.", item_name.c_str());
          continue;
        }

        if (item->memory_type_ == EEPROM)
        {
          uint8_t   data8 = 0;
          uint16_t  data16 = 0;
          uint32_t  data32 = 0;

          switch (item->data_length_)
          {
          case 1:
            read1Byte(joint_name, item->address_, &data8);
            if (data8 == value)
              continue;
            break;
          case 2:
            read2Byte(joint_name, item->address_, &data16);
            if (data16 == value)
              continue;
            break;
          case 4:
            read4Byte(joint_name, item->address_, &data32);
            if (data32 == value)
              continue;
            break;
          default:
            break;
          }
        }

        switch (item->data_length_)
        {
        case 1:
          write1Byte(joint_name, item->address_, (uint8_t) value);
          break;
        case 2:
          write2Byte(joint_name, item->address_, (uint16_t) value);
          break;
        case 4:
          write4Byte(joint_name, item->address_, value);
          break;
        default:
          break;
        }

        if (item->memory_type_ == EEPROM)
        {
          // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
          usleep(item->data_length_ * 55 * 1000);
        }
      }
    }
  } catch (const std::exception& e)
  {
    RCLCPP_INFO(robot_node_->get_logger(),"Dynamixel Init file not found.");
  }

  // [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
  for (auto& it : robot_->ports_)
  {
    if (port_to_bulk_read_[it.first] != 0)
      port_to_bulk_read_[it.first]->clearParam();
  }
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (dxl == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

//    // bulk read default : present position
//    if(dxl->present_position_item != 0)
//    {
//        bulkread_start_addr    = dxl->present_position_item->address;
//        bulkread_data_length   = dxl->present_position_item->data_length;
//    }

    // calculate bulk read start address & data length
    auto indirect_addr_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != dxl->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr_leng = dxl->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            // RCLCPP_WARN(robot_node_->get_logger(),"[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", joint_name.c_str(), indirect_addr, dxl->ctrl_table[dxl->bulk_read_items[i]->item_name]->address + _l);

            read2Byte(joint_name, indirect_addr, &data16);
            if (data16 != dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(joint_name, indirect_addr, dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = dxl->bulk_read_items_[0];

        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr = dxl->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = dxl->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

//    RCLCPP_WARN(robot_node_->get_logger(),"[%12s] start_addr: %d, data_length: %d", joint_name.c_str(), bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[dxl->port_name_]->addParam(dxl->id_, bulkread_start_addr, bulkread_data_length);

    // Torque ON
    if (writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1) != COMM_SUCCESS)
      writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1);
  }

  for (auto& it : robot_->sensors_)
  {
    std::string sensor_name = it.first;
    Sensor     *sensor      = it.second;

    if (sensor == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    // calculate bulk read start address & data length
    auto indirect_addr_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != sensor->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr_leng = sensor->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
//            RCLCPP_WARN(robot_node_->get_logger(),"[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", sensor_name.c_str(), indirect_addr, sensor->ctrl_table[sensor->bulk_read_items[i]->item_name]->address + _l);
            read2Byte(sensor_name, indirect_addr, &data16);
            if (data16 != sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(sensor_name,
                         indirect_addr,
                         sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = sensor->bulk_read_items_[0];

        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr = sensor->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = sensor->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    //RCLCPP_WARN(robot_node_->get_logger(),"[%12s] start_addr: %d, data_length: %d", sensor_name.c_str(), bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[sensor->port_name_]->addParam(sensor->id_, bulkread_start_addr, bulkread_data_length);
  }
}

void RobotisController::gazeboTimerThread()
{
  rclcpp::Rate gazebo_rate(1000 / robot_->getControlCycle());

  while (!stop_timer_)
  {
    if (init_pose_loaded_ == true)
      process();
    gazebo_rate.sleep();
  }
}

void RobotisController::msgQueueThread()
{
  // g_node->create_subscription<std_msgs::msg::String>("wxx_topic", topic_callback);
  // rclcpp::Subscription<robotis_controller_msgs::msg::WriteControlTable>::SharedPtr write_control_table_sub = robot_node_->create_subscription<robotis_controller_msgs::msg::WriteControlTable>(
  //                 "/robotis/write_control_table", RobotisController::writeControlTableCallback);

  auto write_control_table_sub = robot_node_->create_subscription<robotis_controller_msgs::msg::WriteControlTable>(
                                  "/robotis/write_control_table", std::bind(&RobotisController::writeControlTableCallback, this, std::placeholders::_1));

  auto sync_write_item_sub     = robot_node_->create_subscription<robotis_controller_msgs::msg::SyncWriteItem>(
                                  "/robotis/sync_write_item", std::bind(&RobotisController::syncWriteItemCallback, this, std::placeholders::_1));

  auto joint_ctrl_modules_sub  = robot_node_->create_subscription<robotis_controller_msgs::msg::JointCtrlModule>(
                                  "/robotis/set_joint_ctrl_modules", std::bind(&RobotisController::setJointCtrlModuleCallback, this, std::placeholders::_1));
                      
  auto enable_ctrl_module_sub  = robot_node_->create_subscription<std_msgs::msg::String>(
                                  "/robotis/enable_ctrl_module", std::bind(&RobotisController::setCtrlModuleCallback, this, std::placeholders::_1));

  auto control_mode_sub        = robot_node_->create_subscription<std_msgs::msg::String>(
                                  "/robotis/set_control_mode", std::bind(&RobotisController::setControllerModeCallback, this, std::placeholders::_1));

  auto joint_states_sub        = robot_node_->create_subscription<sensor_msgs::msg::JointState>(
                                  "/robotis/set_joint_states", std::bind(&RobotisController::setJointStatesCallback, this, std::placeholders::_1));
                          
  auto enable_offset_sub       = robot_node_->create_subscription<std_msgs::msg::Bool>(
                                  "/robotis/enable_offset", std::bind(&RobotisController::enableOffsetCallback, this, std::placeholders::_1));

                                  // sensor_msgs::JointState::

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_states_sub;
  if (gazebo_mode_ == true)
    gazebo_joint_states_sub = robot_node_->create_subscription<sensor_msgs::msg::JointState>(
                                  "/" + gazebo_robot_name_ + "/joint_states", std::bind(&RobotisController::gazeboJointStatesCallback, this, std::placeholders::_1));

  //  node->create_publisher<std_msgs::msg::String>("wxx_topic");
   /* publisher */
  goal_joint_state_pub_     = robot_node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/goal_joint_states");
  present_joint_state_pub_  = robot_node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/present_joint_states");
  current_module_pub_       = robot_node_->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/present_joint_ctrl_modules");

  if (gazebo_mode_ == true)
  {
    for (auto& it : robot_->dxls_)
    {
      gazebo_joint_position_pub_[it.first] = robot_node_->create_publisher<std_msgs::msg::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_position/command");
                                            
      gazebo_joint_velocity_pub_[it.first] = robot_node_->create_publisher<std_msgs::msg::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_velocity/command");
                                            
      gazebo_joint_effort_pub_[it.first]   = robot_node_->create_publisher<std_msgs::msg::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_effort/command");
    }
  }

  // auto server = g_node->create_service<TestService>("test", handle_service);
  /* service */
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  // auto get_joint_module_server = robot_node_->create_service<robotis_controller_msgs::srv::GetJointModule>(
  //                                "/robotis/get_present_joint_ctrl_modules", std::bind(&RobotisController::getJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), qos_profile);
  // auto get_joint_module_server = robot_node_->create_service<robotis_controller_msgs::srv::GetJointModule>("/robotis/get_present_joint_ctrl_modules", &RobotisController::getJointCtrlModuleService);

  auto get_joint_module_server = robot_node_->create_service<robotis_controller_msgs::srv::GetJointModule>(
    "/robotis/get_present_joint_ctrl_modules", std::bind(&RobotisController::getJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), qos_profile);
                                              
  auto set_joint_module_server = robot_node_->create_service<robotis_controller_msgs::srv::SetJointModule>(
    "/robotis/set_present_joint_ctrl_modules", std::bind(&RobotisController::setJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), qos_profile);
                                              
  auto set_module_server       = robot_node_->create_service<robotis_controller_msgs::srv::SetModule>(
    "/robotis/set_present_ctrl_modules", std::bind(&RobotisController::setCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), qos_profile);
                                              
  auto load_offset_server      = robot_node_->create_service<robotis_controller_msgs::srv::LoadOffset>(
    "/robotis/load_offset", std::bind(&RobotisController::loadOffsetService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), qos_profile);

  rclcpp::WallRate loop_rate(robot_->getControlCycle() / 1000.0);
  while (rclcpp::ok())
    loop_rate.sleep();
}

void *RobotisController::timerThread(void *param)
{
  RobotisController *controller = (RobotisController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  // RCLCPP_DEBUG(robot_node_->get_logger(),"controller::thread_proc started");

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while (!controller->stop_timer_)
  {
    next_time.tv_sec += (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) % 1000000000;

    controller->process();

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
    if (delta_nsec < -100000)
    {
      if (controller->DEBUG_PRINT == true)
      {
        fprintf(stderr, "[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]",
                         delta_nsec / 1000000.0, (long )next_time.tv_sec, (long )next_time.tv_nsec,
                         (long )curr_time.tv_sec, (long )curr_time.tv_nsec);
      }

      // next_time = curr_time + 3 msec
      next_time.tv_sec = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
      next_time.tv_nsec = (curr_time.tv_nsec + 3000000) % 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
  return 0;
}

void RobotisController::startTimer()
{
  if (this->is_timer_running_ == true)
    return;

  if (this->gazebo_mode_ == true)
  {
    // create and start the thread
    gazebo_thread_ = boost::thread(boost::bind(&RobotisController::gazeboTimerThread, this));
  }
  else
  {
    initializeSyncWrite();

    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }

    usleep(8 * 1000);

    int error;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if (error != 0)
      RCLCPP_ERROR(robot_node_->get_logger(),"pthread_attr_setschedpolicy error = %d\n", error);
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (error != 0)
      RCLCPP_ERROR(robot_node_->get_logger(),"pthread_attr_setinheritsched error = %d\n", error);

    memset(&param, 0, sizeof(param));
    param.sched_priority = 31;    // RT
    error = pthread_attr_setschedparam(&attr, &param);
    if (error != 0)
      RCLCPP_ERROR(robot_node_->get_logger(),"pthread_attr_setschedparam error = %d\n", error);

    // create and start the thread
    if ((error = pthread_create(&this->timer_thread_, &attr, this->timerThread, this)) != 0)
    {
      RCLCPP_ERROR(robot_node_->get_logger(),"Creating timer thread failed!!");
      exit(-1);
    }
  }

  this->is_timer_running_ = true;
}

void RobotisController::stopTimer()
{
  int error = 0;

  // set the flag to stop the thread
  if (this->is_timer_running_)
  {
    this->stop_timer_ = true;

    if (this->gazebo_mode_ == false)
    {
      // wait until the thread is stopped.
      if ((error = pthread_join(this->timer_thread_, NULL)) != 0)
        exit(-1);

      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
          it.second->rxPacket();
      }

      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
    }
    else
    {
      // wait until the thread is stopped.
      gazebo_thread_.join();
    }

    this->stop_timer_ = false;
    this->is_timer_running_ = false;
  }
}

bool RobotisController::isTimerRunning()
{
  return this->is_timer_running_;
}

void RobotisController::loadOffset(const std::string path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_WARN(robot_node_->get_logger(),"Fail to load offset yaml.");
    return;
  }

  YAML::Node offset_node = doc["offset"];
  if (offset_node.size() == 0)
    return;

  RCLCPP_INFO(robot_node_->get_logger(),"Load offsets...");
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); it++)
  {
    std::string joint_name = it->first.as<std::string>();
    double offset = it->second.as<double>();

    auto dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl_it->second->dxl_state_->position_offset_ = offset;
  }
}




void RobotisController::writeControlTableCallback(const robotis_controller_msgs::msg::WriteControlTable::SharedPtr msg)
{
  Device *device = NULL;

  if (DEBUG_PRINT)
    fprintf(stderr, "[WriteControlTable] led control msg received\n");

  auto dev_it1 = robot_->dxls_.find(msg->joint_name);
  if(dev_it1 != robot_->dxls_.end())
  {
    device = dev_it1->second;
  }
  else
  {
    auto dev_it2 = robot_->sensors_.find(msg->joint_name);
    if(dev_it2 != robot_->sensors_.end())
    {
      device = dev_it2->second;
    }
    else
    {
      RCLCPP_WARN(robot_node_->get_logger(),"[WriteControlTable] Unknown device : %s", msg->joint_name.c_str());
      return;
    }
  }
}

void RobotisController::syncWriteItemCallback(const robotis_controller_msgs::msg::SyncWriteItem::SharedPtr msg)
{
  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    Device           *device;

    auto d_it1 = robot_->dxls_.find(msg->joint_name[i]);
    if (d_it1 != robot_->dxls_.end())
    {
      device = d_it1->second;
    }
    else
    {
      auto d_it2 = robot_->sensors_.find(msg->joint_name[i]);
      if (d_it2 != robot_->sensors_.end())
      {
        device = d_it2->second;
      }
      else
      {
        RCLCPP_WARN(robot_node_->get_logger(),"[SyncWriteItem] Unknown device : %s", msg->joint_name[i].c_str());
        continue;
      }
    }

//    ControlTableItem *item  = device->ctrl_table_[msg->item_name];
    ControlTableItem *item  = NULL;
    auto item_it = device->ctrl_table_.find(msg->item_name);
    if(item_it != device->ctrl_table_.end())
    {
      item = item_it->second;
    }
    else
    {
      RCLCPP_WARN(robot_node_->get_logger(),"SyncWriteItem] Unknown item : %s", msg->item_name.c_str());
      continue;
    }

    dynamixel::PortHandler   *port           = robot_->ports_[device->port_name_];
    dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

    if (item->access_type_ == Read)
      continue;

    queue_mutex_.lock();

    int idx = 0;
    if (direct_sync_write_.size() == 0)
    {
      direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
      idx = 0;
    }
    else
    {
      for (idx = 0; idx < direct_sync_write_.size(); idx++)
      {
        if (direct_sync_write_[idx]->getPortHandler() == port && direct_sync_write_[idx]->getPacketHandler() == packet_handler)
          break;
      }

      if (idx == direct_sync_write_.size())
        direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
    }

    uint8_t *data = new uint8_t[item->data_length_];
    if (item->data_length_ == 1)
      data[0] = (uint8_t) msg->value[i];
    else if (item->data_length_ == 2)
    {
      data[0] = DXL_LOBYTE((uint16_t )msg->value[i]);
      data[1] = DXL_HIBYTE((uint16_t )msg->value[i]);
    }
    else if (item->data_length_ == 4)
    {
      data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
      data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
    }
    direct_sync_write_[idx]->addParam(device->id_, data);
    delete[] data;

    queue_mutex_.unlock();
  }
}
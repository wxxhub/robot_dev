#ifndef ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_
#define ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/msg/write_control_table.h"
#include "robotis_controller_msgs/msg/sync_write_item.h"
#include "robotis_controller_msgs/msg/joint_ctrl_module.h"
#include "robotis_controller_msgs/srv/get_joint_module.h"
#include "robotis_controller_msgs/srv/set_joint_module.h"
#include "robotis_controller_msgs/srv/set_module.h"
#include "robotis_controller_msgs/srv/load_offset.h"

#include "robotis_device/robot.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_framework_common/sensor_module.h"
#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_sync_write.h"

namespace robotis_framework
{

enum ControllerMode
{
  MotionModuleMode,
  DirectControlMode
};

class RobotisController : public Singleton<RobotisController>
{
private:
  boost::thread queue_thread_;
  boost::thread gazebo_thread_;
  boost::thread set_module_thread_;
  boost::mutex  queue_mutex_;

  bool      init_pose_load_;
  bool      is_timer_running_;
  bool      is_offset_enabled_;
  bool      stop_timer_;
  pthread_t timer_thread_;
  ControllerMode  controller_mode_;

  std::list<MotionModule *> motion_modules_;
  std::list<SensorModule *> sensor_modules_;
  std::vector<dynamixel::GroupSyncWrite *> direct_sync_write_;

  std::map<std::string, double> sensor_result_;

  void gazeboTimerThread();
  void msgQueueThread();
  void setCtrlModuleThread(std::string ctrl_module);
  void setJointCtrlModuleThread();

  bool isTimerStopped();
  void initializeSyncWrite();

public:
  bool  DEBUG_PRINT;
  Robot *robot_;

  bool        gazebo_mode_;
  std::string gazebo_robot_name_;

  /* bulk read */
  std::map<std::string, dynamixel::GroupBulkRead *>   port_to_bulk_read_;

  /* sync write */
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_current_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_p_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_i_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_d_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_p_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_i_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_d_gain_;



  RobotisController();
};

}  // namespace robotis_controller

#endif  // ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_

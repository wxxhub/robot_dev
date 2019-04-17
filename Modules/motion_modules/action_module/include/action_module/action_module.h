#ifndef _ACTION_MODULE__ACTION_MODULE_H_
#define _ACTION_MODULE__ACTION_MODULE_H_

#define _USE_MATH_DEFINES

#include <cmath>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "action_module_msgs/srv/is_running.hpp"
#include "action_module_msgs/msg/start_action.hpp"
#include "robotis_framework_common/motion_module.h"
#include "action_module/visibility_control.h"
#include "action_file_define.h"

namespace robotis_op
{

class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
public:
  ActionModule();
  virtual ~ActionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  bool loadFile(std::string file_name);
  bool createFile(std::string file_name);

  bool start(int page_number);
  bool start(std::string page_name);
  bool start(int page_number, action_file_define::Page* page);

  void onModuleEnable();
  void onModuleDisable();

  void brake();
  bool isRunning(int* playing_page_num, int* playing_step_num);
  bool loadPage(int page_number, action_file_define::Page* page);
  bool savePage(int page_number, action_file_define::Page* page);
  void resetPage(action_file_define::Page* page);

  void enableAllJoints();
  void actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls);

private:
  const int PRE_SECTION;
  const int MAIN_SECTION;
  const int POST_SECTION;
  const int PAUSE_SECTION;
  const int ZERO_FINISH;
  const int NONE_ZERO_FINISH;
  const bool DEBUG_PRINT;

  void queueThread();

  bool verifyChecksum( action_file_define::Page* page );
  void setChecksum( action_file_define::Page* page );

  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);

  bool isRunningServiceCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<action_module_msgs::srv::IsRunning::Request> req,
                                const std::shared_ptr<action_module_msgs::srv::IsRunning::Response> res);

  void pageNumberCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void startActionCallback(const action_module_msgs::msg::StartAction::SharedPtr msg);

  int convertRadTow4095(double rad);
  double convertw4095ToRad(int w4095);
  std::string convertIntToString(int n);

  std::map<std::string, bool> action_joints_enable_;
  std::map<std::string, robotis_framework::DynamixelState *> action_result_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;

  rclcpp::Node::SharedPtr module_node_;
  rclcpp::Clock ros_clock_;
  /* sample subscriber & publisher */
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  done_msg_pub_;
  /////////////////////////////////////////////////////////////////////////
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  FILE* action_file_;
  action_file_define::Page play_page_;
  action_file_define::Page next_play_page_;
  action_file_define::Step current_step_;

  int play_page_idx_;
  bool first_driving_start_;
  int page_step_count_;

  bool playing_;
  bool stop_playing_;
  bool playing_finished_;

  bool action_module_enabled_;
  bool previous_running_;
  bool present_running_;
};

}  // namespace robotis_op

#endif  // _ACTION_MODULE__ACTION_MODULE_H_

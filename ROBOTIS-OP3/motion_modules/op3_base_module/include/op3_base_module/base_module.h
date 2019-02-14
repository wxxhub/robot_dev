#ifndef OP3_BASE_MODULE__BASE_MODULE_HPP_
#define OP3_BASE_MODULE__BASE_MODULE_HPP_

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "robotis_framework_common/motion_module.h"
#include "op3_base_module/visibility_control.h"
#include "base_module_state.h"

namespace robotis_op
{

class BaseJointData
{
 public:
  double position_;
  double velocity_;
  double effort_;

  int p_gain_;
  int i_gain_;
  int d_gain_;

};

class BaseJointState
{

 public:
  BaseJointData curr_joint_state_[ MAX_JOINT_ID + 1];
  BaseJointData goal_joint_state_[ MAX_JOINT_ID + 1];
  BaseJointData fake_joint_state_[ MAX_JOINT_ID + 1];

};

class BaseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
public:
  BaseModule();

  virtual ~BaseModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg);

  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();

  void poseGenerateProc(Eigen::MatrixXd joint_angle_pose);
  void poseGenerateProc(std::map<std::string, double>& joint_angle_pose);

  /* Parameter */
  BaseModuleState *base_module_state_;
  BaseJointState *joint_state_;

 private:
  void queueThread();
  void setCtrlModule(std::string module);
  void callServiceSettingModule(const std::string &module_name);
  void parseInitPoseData(const std::string &path);
  void publishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;

  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr  status_msg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  set_ctrl_module_pub_;

  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;

  std::map<std::string, int> joint_name_to_id_;

  bool has_goal_joints_;
  bool ini_pose_only_;

  std::string	init_pose_file_path_;

  rclcpp::Node::SharedPtr module_node_;
};

}  // namespace op3_base_module

#endif  // OP3_BASE_MODULE__BASE_MODULE_HPP_

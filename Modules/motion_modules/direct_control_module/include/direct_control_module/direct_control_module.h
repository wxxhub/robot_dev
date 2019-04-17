#ifndef _DIRECT_CONTROL_MODULE__DIRECT_CONTROL_MODULE_H_
#define _DIRECT_CONTROL_MODULE__DIRECT_CONTROL_MODULE_H_

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "direct_control_module/visibility_control.h"

namespace robotis_op
{

class DirectControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<DirectControlModule>
{
public:
  DirectControlModule();

  virtual ~DirectControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

private:
  enum TraIndex
  {
    Position = 0,
    Velocity = 1,
    Acceleration = 2,
    Count
  };

  const int BASE_INDEX;
  const int HEAD_INDEX;
  const int RIGHT_END_EFFECTOR_INDEX;
  const int RIGHT_ELBOW_INDEX;
  const int LEFT_END_EFFECTOR_INDEX;
  const int LEFT_ELBOW_INDEX;

  /* ROS Topic Callback Functions */
  void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void queueThread();
  void jointTraGeneThread();

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                        double vel_end, double accel_end, double smp_time, double mov_time);

  std::map<std::string, bool> collision_;

  bool checkSelfCollision();
  bool getDiff(OP3KinematicsDynamics *kinematics, int end_index, int base_index, double &diff);

  double default_moving_time_;
  double default_moving_angle_;
  bool check_collision_;

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_updated_;
  bool is_blocked_;
  bool will_be_collision_;
  int tra_count_, tra_size_;
  double moving_time_;
  double r_min_diff_, l_min_diff_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd present_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;
  std::map<int, double> max_angle_;
  std::map<int, double> min_angle_;

  rclcpp::Node::SharedPtr module_node_;
  rclcpp::Clock ros_clock_;
  rclcpp::Time last_msg_time_;
  std::string last_msg_;

  OP3KinematicsDynamics *op3_kinematics_;
};

}  // namespace direct_control_module

#endif  // _DIRECT_CONTROL_MODULE__DIRECT_CONTROL_MODULE_H_

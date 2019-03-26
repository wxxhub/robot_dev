#ifndef OP3_HEAD_CONTROL_MODULE__HEAD_CONTROL_MODULE_HPP_
#define OP3_HEAD_CONTROL_MODULE__HEAD_CONTROL_MODULE_HPP_

#include <cstdlib>
#include <ctime>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "op3_head_control_module/visibility_control.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace robotis_op
{

class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
{
public:
  HeadControlModule();
  ~HeadControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

 private:
  enum
  {
    NoScan = 0,
    TopLeft = 1,
    BottomRight = 2,
    BottomLeft = 3,
    TopRight = 4,
  };

  void setHeadJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void setHeadJointOffsetCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void setHeadScanCallback(const std_msgs::msg::String::SharedPtr msg);

  void queueThread();
  void jointTraGeneThread();
  void setHeadJoint(const sensor_msgs::msg::JointState::SharedPtr msg, bool is_offset);
  bool checkAngleLimit(const int joint_index, double &goal_position);
  void generateScanTra(const int head_direction);

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                        double vel_end, double accel_end, double smp_time, double mov_time);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;
  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_direct_control_;
  int tra_count_, tra_size_;
  double moving_time_;
  int scan_state_;
  bool has_goal_position_;
  double angle_unit_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
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
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Clock ros_clock;
  rclcpp::Time last_msg_time_;
  std::string last_msg_;
};

}  // namespace op3_head_control_module

#endif  // OP3_HEAD_CONTROL_MODULE__HEAD_CONTROL_MODULE_HPP_

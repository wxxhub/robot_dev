#ifndef _WALKING_MODULE__WALKING_MODULE_H_
#define _WALKING_MODULE__WALKING_MODULE_H_

#include "walking_module/visibility_control.h"

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <eigen_conversions/eigen_msg.h>


#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "walking_module_msgs/msg/walking_param.hpp"
#include "walking_module_msgs/srv/get_walking_param.hpp"
#include "walking_module_msgs/srv/set_walking_param.hpp"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{
public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

  WalkingModule();
  virtual ~WalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();
  void onModuleEnable();
  void onModuleDisable();

  int getCurrentPhase()
  {
    return phase_;
  }
  double getBodySwingY()
  {
    return body_swing_y;
  }
  double getBodySwingZ()
  {
    return body_swing_z;
  }

private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };

  const bool DEBUG;

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void walkingParameterCallback(const walking_module_msgs::msg::WalkingParam::SharedPtr msg);
  bool getWalkigParameterCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<walking_module_msgs::srv::GetWalkingParam::Request> req,
                                  const std::shared_ptr<walking_module_msgs::srv::GetWalkingParam::Response> res);

  void balanceParameterCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /* ROS Service Callback Functions */
  void processPhase(const double &time_unit);
  bool computeLegAngle(double *leg_angle);
  void computeArmAngle(double *arm_angle);
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  void updateTimeParam();
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);
  void iniPoseTraGene(double mov_time);

  OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  // rclcpp::Publisher::SharedPtr robot_pose_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr step_number_msg_pub_;
  rclcpp::Node::SharedPtr module_node_;
  rclcpp::Clock ros_clock_;

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;
  walking_module_msgs::msg::WalkingParam walking_param_;
  double previous_x_move_amplitude_;

  // variable for walking
  double period_time_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_period_time_;
  double x_move_period_time_;
  double y_swap_period_time_;
  double y_move_period_time_;
  double z_swap_period_time_;
  double z_move_period_time_;
  double a_move_period_time_;
  double ssp_time_;
  double l_ssp_start_time_;
  double l_ssp_end_time_;
  double r_ssp_start_time_;
  double r_ssp_end_time_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;

  double x_offset_;
  double y_offset_;
  double z_offset_;
  double r_offset_;
  double p_offset_;
  double a_offset_;

  double x_swap_phase_shift_;
  double x_swap_amplitude_;
  double x_swap_amplitude_shift_;
  double x_move_phase_shift_;
  double x_move_amplitude_;
  double x_move_amplitude_shift_;
  double y_swap_phase_shift_;
  double y_swap_amplitude_;
  double y_swap_amplitude_shift_;
  double y_move_phase_shift_;
  double y_move_amplitude_;
  double y_move_amplitude_shift_;
  double z_swap_phase_shift_;
  double z_swap_amplitude_;
  double z_swap_amplitude_shift_;
  double z_move_phase_shift_;
  double z_move_amplitude_;
  double z_move_amplitude_shift_;
  double a_move_phase_shift_;
  double a_move_amplitude_;
  double a_move_amplitude_shift_;

  double pelvis_offset_;
  double pelvis_swing_;
  double hit_pitch_offset_;
  double arm_swing_gain_;

  bool ctrl_running_;
  bool real_running_;
  double time_;

  int phase_;
  double body_swing_y;
  double body_swing_z;

  int step_number_;

  float test_balance_ = 0;
};

}  // namespace walking_module

#endif  // _WALKING_MODULE__WALKING_MODULE_H_

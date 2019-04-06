#include "base_module/base_module_state.h"

namespace robotis_op
{

BaseModuleState::BaseModuleState()
{
  is_moving_ = false;

  cnt_ = 0;

  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);
  joint_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  via_num_ = 1;

  joint_via_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_dpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);
}

BaseModuleState::~BaseModuleState()
{
}

}

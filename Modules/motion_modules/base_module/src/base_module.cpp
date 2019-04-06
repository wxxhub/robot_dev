#include "base_module/base_module.h"

#include <iostream>
// #include <io.h>


const std::string NONE_STRING = "";
const std::string INIT_PATH = "/data/ini_pose.yaml";
rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

namespace robotis_op
{

BaseModule::BaseModule()
	: control_cycle_msec_(0),
		has_goal_joints_(true),
    ini_pose_only_(false),
    init_pose_file_path_("")
{
	enable_ = true;
	module_name_ = "base_module";
  control_mode_ = robotis_framework::PositionControl;
  module_node_ = rclcpp::Node::make_shared(module_name_);

  base_module_state_ = new BaseModuleState();
  joint_state_ = new BaseJointState();
}

BaseModule::~BaseModule()
{
	queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	std::cout<<"base module init"<<std::endl;
	char   buffer[80];
	getcwd(buffer, 80);
	std::cout << "base module run path" << buffer << std::endl;
	control_cycle_msec_ = control_cycle_msec;

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  init_pose_file_path_ = ament_index_cpp::get_package_share_directory("base_module") + "/data/ini_pose.yaml";
	std::cout<<"init_pose_file_path_: "<<init_pose_file_path_<<std::endl;

	/* publish topics */
  status_msg_pub_ = module_node_->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status");
  set_ctrl_module_pub_ = module_node_->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module");
  set_module_client_ = module_node_->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/load_offset");

  queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));
}

void BaseModule::queueThread()
{
	auto ini_pose_msg_sub = module_node_->create_subscription<std_msgs::msg::String>("/robotis/base/ini_pose", std::bind(&BaseModule::initPoseMsgCallback, this, std::placeholders::_1));
	// node->create_client<TestService>("test");

	rclcpp::WallRate loop_rate(1000.0/control_cycle_msec_);
  // rclcpp::WallRate loop_rate(60);
	while (rclcpp::ok())
	{
    rclcpp::spin_some(module_node_);
    // std::cout<<"spin_some ..."<<std::endl;
		loop_rate.sleep();
	}
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;
  // printf("base module prcessing...\n");
  /*----- write curr position -----*/
  std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter;
  for (state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it;
    dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;
    
    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */
  if (base_module_state_->is_moving_ == true)
  {
    if (base_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

    base_module_state_->cnt_++;
  }

  /*----- set joint data -----*/
  for (state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    // printf("%s goal_position: %d\n", joint_name.c_str(), joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_);
    // result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
    result_[joint_name]->goal_position_ = 0;
  } 

  /*---------- initialize count number ----------*/

  if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
  {
    RCLCPP_INFO(module_node_->get_logger(), "[end] send trajectory");
    publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Finish Init Pose");

    base_module_state_->is_moving_ = false;
    base_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void BaseModule::stop()
{
  return;
}

bool BaseModule::isRunning()
{
  return base_module_state_->is_moving_;
}

void BaseModule::onModuleEnable()
{
  RCLCPP_INFO(module_node_->get_logger(), "Base Module is enabled");
}

void BaseModule::onModuleDisable()
{
  has_goal_joints_ = false;
}

void BaseModule::initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  printf("### init\n");
	if (base_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      callServiceSettingModule(module_name_);

      // wait for changing the module to base_module and getting the goal position
      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      // parse initial pose
      parseInitPoseData(init_pose_file_path_);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::initPoseTrajGenerateProc, this));
    }
  }
  else
    RCLCPP_INFO(module_node_->get_logger(), "previous task is alive");

  // return;
}

void BaseModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (base_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  base_module_state_->smp_time_, base_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = base_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = base_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = base_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(base_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, base_module_state_->smp_time_,
                                                               base_module_state_->via_time_,
                                                               base_module_state_->mov_time_);
    }
    
    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  RCLCPP_INFO(module_node_->get_logger(), "[start] send trajectory");
}

void BaseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  base_module_state_->mov_time_ = 1.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  base_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    // ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);
    RCLCPP_INFO(module_node_->get_logger(), "[ID ： %d] ini_value : %f  tar_value : %f", id, ini_value, tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  RCLCPP_INFO(module_node_->get_logger(), "[start] send trajectory");
}

void BaseModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  for (std::map<std::string, double>::iterator joint_angle_it = joint_angle_pose.begin();
       joint_angle_it != joint_angle_pose.end(); joint_angle_it++)
  {
    std::string joint_name = joint_angle_it->first;
    double joint_angle_rad = joint_angle_it->second;

    std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
    if (joint_name_to_id_it != joint_name_to_id_.end())
    {
      target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
    }
  }

  base_module_state_->joint_pose_ = target_pose;

  base_module_state_->mov_time_ = 5.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    // ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);
    RCLCPP_INFO(module_node_->get_logger(), "[ID ： %d] ini_value : %f  tar_value : %f", id, ini_value, tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  RCLCPP_INFO(module_node_->get_logger(), "[start] send trajectory");
}

void BaseModule::setCtrlModule(std::string module)
{
  std_msgs::msg::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_->publish(control_msg);
}

void BaseModule::callServiceSettingModule(const std::string &module_name)
{
  // auto set_module_srv = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  // set_module_srv->module_name = module_name;

  // auto result_future = set_module_client_->async_send_request(set_module_srv);
  // if (rclcpp::spin_until_future_complete(module_node_, result_future) != 
  //       rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(module_node_->get_logger(), "Failed to set module");
  //   return;
  // }

  return ;
}

void BaseModule::parseInitPoseData(const std::string &path)
{
  printf("parseInitPoseData\n");
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(module_node_->get_logger(), "Fail to load yaml file.");
    return;
  }

  // printf("test1\n");
  // parse movement time
  double mov_time;
  mov_time = doc["mov_time"].as<double>();

  base_module_state_->mov_time_ = mov_time;

  // parse via-point number
  int via_num;
  via_num = doc["via_num"].as<int>();

  base_module_state_->via_num_ = via_num;

  // printf("test1\n");
  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  base_module_state_->via_time_.resize(via_num, 1);
  for (int num = 0; num < via_num; num++)
    base_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  base_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  base_module_state_->joint_via_pose_.fill(0.0);
  base_module_state_->joint_via_dpose_.fill(0.0);
  base_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
  {
    int id;
    // printf("test3\n");
    std::vector<double> value;

    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num; num++)
      base_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    int id;
    double value;

    // printf("test4\n");
    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<double>();

    base_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = ros_clock.now();
  status_msg.type = type;
  status_msg.module_name = "Base";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

}  // namespace robotis_op

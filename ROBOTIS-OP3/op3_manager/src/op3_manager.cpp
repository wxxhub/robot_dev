
/* Author: wxx */
#include <iostream>
/* ROS API Header */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

// /* Sensor Module Header */
// #include "open_cr_module/open_cr_module.h"

// /* Motion Module Header */
// #include "op3_action_module/action_module.h"

// /* Regulator Module Header */
// #include "hand_regulator_module/hand_regulator_module.h"

using namespace robotis_framework;
using namespace dynamixel;
// using namespace robotis_op;

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;

bool g_is_simulation = false;
int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

// ros::Publisher g_init_pose_pub;
// ros::Publisher g_demo_command_pub;

void parseInitPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    printf("Fail to load yaml file.");
    return;
  }

  // parse movement time
  int mov_time;
  mov_time = doc["test"].as<double>();
  std::cout<<"test: "<<mov_time<<std::endl;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto manager_node = rclcpp::Node::make_shared("op3_manager");

  RCLCPP_INFO(manager_node->get_logger(),"manager init");


  RobotisController *controller = RobotisController::getInstance();

  manager_node->get_parameter("offset_file_path", g_offset_file);
  std::cout<<"offset_file_path: "<<g_offset_file<<std::endl;
  parseInitPoseData(g_offset_file);
  // printf("offset_file_path:%d\n", g_offset_file);

  printf("finished\n");
  rclcpp::shutdown();
  return 0;
}

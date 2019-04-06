
/* Author: wxx */
#include <iostream>
#include <chrono>
/* ROS API Header */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

// /* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

// /* Motion Module Header */
#include "action_module/action_module.h"
#include "base_module/base_module.h"

// /* Regulator Module Header */
#include "body_regulator_module/body_regulator_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;
using namespace std::chrono_literals;

rclcpp::Node::SharedPtr manager_node = nullptr;

const int BAUD_RATE = 1000000;
const double PROTOCOL_VERSION = 1.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const int DEVICE_NUMBER = 4; //串口数量
const std::string SUB_CONTROLLER_DEVICE0 = "/dev/ttyUSB0";
const std::string SUB_CONTROLLER_DEVICE1 = "/dev/ttyUSB1";
const std::string SUB_CONTROLLER_DEVICE2 = "/dev/ttyUSB2";
const std::string SUB_CONTROLLER_DEVICE3 = "/dev/ttyUSB3";
const std::string NONE_STRING = "";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;

bool g_is_simulation = false;
int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;


rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_demo_command_pub;

void dxlTorqueCheckCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(manager_node->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char ** argv)
{
  char   buffer[80];
	getcwd(buffer, 80);
	std::cout << "manager run path: " << buffer << std::endl;

  rclcpp::init(argc, argv);
  manager_node = rclcpp::Node::make_shared("module_manager");

  RCLCPP_INFO(manager_node->get_logger(),"manager init");


  RobotisController *controller = RobotisController::getInstance();

  manager_node->get_parameter_or("offset_file_path", g_offset_file, NONE_STRING);
  manager_node->get_parameter_or("robot_file_path", g_robot_file, NONE_STRING);
  manager_node->get_parameter_or("init_file_path", g_init_file, NONE_STRING);
  manager_node->get_parameter_or("device_name",g_device_name, SUB_CONTROLLER_DEVICE0);
  manager_node->get_parameter_or("baud_rate", g_baudrate, BAUD_RATE);
  manager_node->get_parameter_or("gazebo", controller->gazebo_mode_, false);
  std::cout<<"g_device_name: "<<g_device_name<<std::endl;
  std::cout<<"BAUD_RATE: "<<BAUD_RATE<<std::endl;
  // printf("g_device_name:%s\n", g_device_name);

  /* ros2 message */
  auto subscription = manager_node->create_subscription<std_msgs::msg::String>
      ("topic", dxlTorqueCheckCallback);

  g_init_pose_pub = manager_node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose");
  g_is_simulation = controller->gazebo_mode_;

  if (g_is_simulation == false)
  {
    PortHandler *port_handler;
    bool set_port_result = false;
    int set_port_times = 0;
    while (set_port_result == false)
    {
      port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
      set_port_result = port_handler->setBaudRate(BAUD_RATE);
      if (set_port_result == false)
      {
        RCLCPP_ERROR(manager_node->get_logger(),"Error Set port! change to device %d", set_port_times%DEVICE_NUMBER);
        set_port_times++;
        switch (set_port_times%DEVICE_NUMBER)
        {
          case 0:
            g_device_name = SUB_CONTROLLER_DEVICE0;
            break;

          case 1:
            g_device_name = SUB_CONTROLLER_DEVICE1;
            break;

          case 2:
            g_device_name = SUB_CONTROLLER_DEVICE2;
            break;

          case 3:
            g_device_name = SUB_CONTROLLER_DEVICE3;
            break;
        
          default:
            RCLCPP_ERROR(manager_node->get_logger(),"please reset DEVICE_NUMBER");
            break;
        }
        usleep(1000);
      }

      if (set_port_times > 1000)
      {
        RCLCPP_ERROR(manager_node->get_logger(),"Error Set port! please check device!");
        return -1;
      }
    }

    PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if (_return != 0)
      {
        RCLCPP_ERROR(manager_node->get_logger(), "Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
      }
      else
        RCLCPP_INFO(manager_node->get_logger(), "Torque on DXLs!");

      if (_return == 0)
        break;
      else
        torque_on_count++;
    }

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packet_handler->write2ByteTxRx(port_handler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
       RCLCPP_ERROR(manager_node->get_logger(), "Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

    port_handler->closePort();
  }
  else
  {
    RCLCPP_WARN(manager_node->get_logger(), "SET TO GAZEBO MODE!");
    std::string robot_name;
    manager_node->get_parameter_or("offset_file_path", g_offset_file, NONE_STRING);
    if (robot_name != NONE_STRING)
      controller->gazebo_robot_name_ = robot_name;
  }

  if (g_robot_file == "")
  {
    RCLCPP_ERROR(manager_node->get_logger(), "NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  RCLCPP_INFO(manager_node->get_logger(), "initialize robot");
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    RCLCPP_ERROR(manager_node->get_logger(), "ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  RCLCPP_INFO(manager_node->get_logger(), "load offset");
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);
  usleep(300 * 1000);
  
//  RCLCPP_INFO(manager_node->get_logger(), "start add module");

  /* Add Sensor Module */
  // controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  /* Add Motion Module */
  // controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  // controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
  // controller->addMotionModule((MotionModule*) WalkingModule::getInstance());
  // controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());

  /* Add Regulator Module */
  controller->addRegulatorModule((RegulatorModule*) BodyRegulatorModule::getInstance());

  RCLCPP_INFO(manager_node->get_logger(), "finished add module");

  controller->startTimer();

  usleep(100 * 1000);

  // go to init pose
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub->publish(init_msg);
  RCLCPP_INFO(manager_node->get_logger(), "Go to init pose");

  rclcpp::WallRate loop_rate(60);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(manager_node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}

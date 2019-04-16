#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "behaviors/behaviors.h"

using namespace robot_behavior;
using namespace std;

enum Behaviors_Status
{
  Ready = 0,
  Behavior_Demo = 1,
};


rclcpp::Node::SharedPtr manager_node = nullptr;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_mode_pub;

int current_status = Ready;
int desired_status = Ready;
bool apply_desired = true;
std::string robot_manager_name = "module_manager";

void goInitPose();
bool checkRobotManagerRunning(std::string& manager_name);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  manager_node = rclcpp::Node::make_shared("behavior_manager");
    // create wrapper object
  BehaviorModel *current_behavior = NULL;
  BehaviorDemo  *demo_behavior    = new BehaviorDemo();
  usleep(100 * 1000);
  while (rclcpp::ok())
  {
    if (checkRobotManagerRunning(robot_manager_name) == true)
    {
      RCLCPP_INFO(manager_node->get_logger(), "Succeed to connect %s!", robot_manager_name.c_str());
      usleep(100 * 1000);
      break;
    }
    else
    {
      RCLCPP_WARN(manager_node->get_logger(), "Can't to connect %s!", robot_manager_name.c_str());
    }
    
    rclcpp::WallRate(1).sleep();
  }

  // manager loop
  while (rclcpp::ok())
  {
    // RCLCPP_INFO(manager_node->get_logger(), "running");
    if (apply_desired == true)
    {
      desired_status = Behavior_Demo;

      switch (desired_status)
      {
        case Ready:
        {
          if (current_behavior != NULL)
            current_behavior->setDisable();
          
          current_behavior = NULL;
          goInitPose();
          break;
        }

        case Behavior_Demo:
        {
          if (current_behavior != NULL)
            current_behavior->setDisable();
          current_behavior = demo_behavior;
          current_behavior->setEnable();
          break;
        }
      }

      apply_desired = false;
      current_status = desired_status;
    }
    rclcpp::WallRate(60).sleep();
  }
  return 0;
}

void goInitPose()
{

}

bool checkRobotManagerRunning(std::string& manager_name)
{
  // get_node_names

  vector<string> node_list = manager_node->get_node_names();

  vector<string>::iterator iter = node_list.begin();

  for (; iter != node_list.end(); iter++)
  {
    if (manager_name == *iter)
    {
      usleep(500 * 1000);
      return true;
    }
  }
  usleep(500 * 1000);
  return false;
}
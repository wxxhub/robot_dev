#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "behaviours/behaviours.h"

using namespace robot_behaviour;

enum Behaviours_Status
{
  Ready = 0,
  Behaviour_Demo = 1,
};


rclcpp::Node::SharedPtr manager_node = nullptr;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_mode_pub;

int current_status = Ready;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  manager_node = rclcpp::Node::make_shared("behaviour_manager");

  // create wrapper object
  BehaviourModel *current_behaviour = NULL;
  BehaviourDemo  *demo_behaviour    = new BehaviourDemo();

  printf("hello world behaviour_manager package\n");
  return 0;
}
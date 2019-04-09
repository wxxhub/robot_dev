#ifndef _BEHAVIOUR_DEMO_H_
#define _BEHAVIOUR_DEMO_H_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/thread.hpp>
#include <map>

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"

#include "behaviour_model.h"

namespace robot_behaviour
{

class BehaviourDemo : public robot_behaviour::BehaviourModel
{
public:
    BehaviourDemo();
    ~BehaviourDemo();

    void setEnable();
    void setDisable();

private:
    void startDemo();
    void stopDemo();

    void parseJointName(std::string path);

    void callbackThread();
    void processThread();

    void process();

    void setModuleToDemo(const std::string module_name);
    void callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr modules);

    rclcpp::Node::SharedPtr behaviour_node_;
    rclcpp::Node::SharedPtr behaviour_client_node_;
    rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr module_control_pub_;
    rclcpp::Client<robotis_controller_msgs::srv::SetJointModule>::SharedPtr set_joint_module_client_;

    const int spin_rate_;

    std::map<int, std::string> id_joint_table_;
};

}

#endif  /* _BEHAVIOUR_DEMO_H_ */
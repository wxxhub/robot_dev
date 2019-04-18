#ifndef _BEHAVIOR_MARATHON_H_
#define _BEHAVIOR_MARATHON_H_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/thread.hpp>
#include <map>

#include "std_srvs/srv/set_bool.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include "detector_msgs/msg/road_result.hpp"

#include "behavior_model.h"

namespace robot_behavior
{

enum Direction{
    left = -1,
    direct = 0,
    right = 1,
};

struct RoadInfo
{
    bool mark_exist;
    bool road_exist;
    int up_x;
    int up_y;
    int down_x;
    int down_y;
    int image_width;
    int image_height;
    Direction direction;
};


class Marathon : public robot_behavior::BehaviorModel
{
public:
    Marathon();
    ~Marathon();

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
    void roadDetCallback(const detector_msgs::msg::RoadResult::SharedPtr msg);

    rclcpp::Node::SharedPtr behavior_node_;
    rclcpp::Node::SharedPtr behavior_client_node_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_control_pub_;

    rclcpp::Client<robotis_controller_msgs::srv::SetJointModule>::SharedPtr set_joint_module_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_road_detector_client_;

    rclcpp::Subscription<detector_msgs::msg::RoadResult>::SharedPtr road_sub_;

    const int spin_rate_;

    std::map<int, std::string> id_joint_table_;

    RoadInfo* road_info_;
};

}

#endif /* _BEHAVIOR_MARATHON_H_ */
#include "behaviours/behaviour_demo.h"

using namespace robot_behaviour;
using namespace std;

BehaviourDemo::BehaviourDemo()
    : spin_rate_(60)
{
    // init
    enable_ = false;

    behaviour_node_ = rclcpp::Node::make_shared("behaviour_demo");
    behaviour_client_node_ = rclcpp::Node::make_shared("behaviour_demo_client");
    string default_path = "";
    default_path = ament_index_cpp::get_package_share_directory("behaviours")+"/config/gui_config.yaml";
    string path = "";

    printf("test1\n");
    behaviour_node_->get_parameter_or("demo_config", path, default_path);
    printf("test2\n");
    boost::thread queue_thread = boost::thread(boost::bind(&BehaviourDemo::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&BehaviourDemo::processThread, this));

    parseJointName(path);
}

BehaviourDemo::~BehaviourDemo()
{

}

void BehaviourDemo::parseJointName(string path)
{
    printf("parseJointName\n");
    id_joint_table_[7] = "head_tilt";
    id_joint_table_[11] = "head_pan";
}

void BehaviourDemo::callbackThread()
{
    // module_control_pub_ = behaviour_node_->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/set_joint_ctrl_modules");

    set_joint_module_client_ = behaviour_client_node_->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");
    rclcpp::WallRate loop_rate(spin_rate_);
    while (rclcpp::ok())
    {
        // rclcpp::spin_some(behaviour_node_);

        loop_rate.sleep();
    }
}

void BehaviourDemo::processThread()
{
    bool result = false;

    rclcpp::WallRate loop_rate(spin_rate_);

    // loop
    while (rclcpp::ok())
    {
        if (enable_ == true)
            process();

        loop_rate.sleep();
    }
}

void BehaviourDemo::process()
{

}

void BehaviourDemo::setModuleToDemo(const std::string module_name)
{
    if (enable_ == false)
        return;

    // shared_ptr<robotis_controller_msgs::msg::JointCtrlModule> control_msg;
    auto control_msg = std::make_shared<robotis_controller_msgs::msg::JointCtrlModule>();
    // robotis_controller_msgs::msg::JointCtrlModule control_msg;
    map<int, string>::iterator joint_iter = id_joint_table_.begin();
    
    for (; joint_iter != id_joint_table_.end(); joint_iter++)
    {
        control_msg->joint_name.push_back(joint_iter->second);
        control_msg->module_name.push_back(module_name);
    }

    if (control_msg->joint_name.size() == 0)
        return;

    callServiceSettingModule(control_msg);
}

void BehaviourDemo::callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr modules)
{
    auto set_joint_srv = std::make_shared<robotis_controller_msgs::srv::SetJointModule::Request>();
    set_joint_srv->joint_name = modules->joint_name;
    set_joint_srv->module_name = modules->module_name;

    printf("modules->joint_name.size(): %d\n", modules->joint_name.size());
    printf("set_joint_srv->joint_name.size(): %d\n", set_joint_srv->joint_name.size());
    auto result_future = set_joint_module_client_->async_send_request(set_joint_srv);
    if (rclcpp::spin_until_future_complete(behaviour_client_node_, result_future) != 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(behaviour_client_node_->get_logger(), "Failed to set module");
        return;
    }
}

void BehaviourDemo::setEnable()
{
    enable_ = true;
    startDemo();
}

void BehaviourDemo::setDisable()
{
    stopDemo();
    enable_ = false;
}

void BehaviourDemo::startDemo()
{
    setModuleToDemo("head_control_module");
}

void BehaviourDemo::stopDemo()
{
    
}
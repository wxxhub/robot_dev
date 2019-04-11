#include "behaviours/behaviour_demo.h"

using namespace robot_behaviour;
using namespace std;

BehaviourDemo::BehaviourDemo()
    : spin_rate_(60),
      head_pan_name_("head_pan"),
      head_tilt_name_("head_tilt"),
      head_offset_ratio_(1.0)
{
    // init
    enable_ = false;
    ball_info_.update = false;

    behaviour_node_ = rclcpp::Node::make_shared("behaviour_demo");
    behaviour_client_node_ = rclcpp::Node::make_shared("behaviour_demo_client");
    string default_path = ament_index_cpp::get_package_share_directory("behaviours")+"/config/gui_config.yaml";
    string path = "";

    behaviour_node_->get_parameter_or("demo_config", path, default_path);
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
    id_joint_table_[7] = head_tilt_name_;
    id_joint_table_[11] = head_pan_name_;
}

void BehaviourDemo::callbackThread()
{
    head_control_pub_ = behaviour_node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states_offset");

    ball_sub_ = behaviour_node_->create_subscription<detector_msgs::msg::BallDetector>("/ball_lab_detector/result", std::bind(&BehaviourDemo::ballDetCallback, this, std::placeholders::_1));

    set_joint_module_client_ = behaviour_client_node_->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");
    rclcpp::WallRate loop_rate(spin_rate_);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(behaviour_node_);

        loop_rate.sleep();
    }
}

void BehaviourDemo::ballDetCallback(const detector_msgs::msg::BallDetector::SharedPtr msg)
{
    // printf("image:(%d, %d), ball:(%f, %f, %d)\n", msg->image_width, msg->image_height, msg->center_position.x, msg->center_position.y, msg->radius);
    ball_info_.image_width  = msg->image_width;
    ball_info_.image_height = msg->image_height;
    ball_info_.position_x   = msg->center_position.x;
    ball_info_.position_y   = msg->center_position.y;
    ball_info_.radius       = msg->radius;
    ball_info_.update       = true;
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
    if (!enable_)
        return;
    if (ball_info_.update)
    {
        ball_info_.update = false;
        sensor_msgs::msg::JointState set_offset_msg;

        double x_offset = (ball_info_.position_x - ball_info_.image_width / 2.0) / ball_info_.image_width;
        double y_offset = (ball_info_.position_y - ball_info_.image_height / 2.0) / ball_info_.image_height;

        set_offset_msg.name.push_back(head_pan_name_);
        set_offset_msg.name.push_back(head_tilt_name_);

        set_offset_msg.position.push_back(x_offset * head_offset_ratio_);
        set_offset_msg.position.push_back(y_offset * head_offset_ratio_);

        head_control_pub_->publish(set_offset_msg);
        printf("publised image:(%d, %d), ball:(%d, %d, %d)\n", ball_info_.image_width, ball_info_.image_height, ball_info_.position_x, ball_info_.position_y, ball_info_.radius);
    }
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
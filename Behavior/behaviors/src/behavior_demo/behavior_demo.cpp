#include "behaviors/behavior_demo.h"

using namespace robot_behavior;
using namespace std;

BehaviorDemo::BehaviorDemo()
    : spin_rate_(60),
      head_pan_name_("head_pan"),
      head_tilt_name_("head_tilt"),
      head_offset_ratio_(1.0)
{
    // init
    enable_ = false;
    ball_info_.update = false;

    behavior_node_ = rclcpp::Node::make_shared("behavior_demo");
    behavior_client_node_ = rclcpp::Node::make_shared("behavior_demo_client");

    string default_path = " ";
    try
    {
        default_path = ament_index_cpp::get_package_share_directory("behaviors")+"/config/gui_config.yaml";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    string path = "";

    behavior_node_->get_parameter_or("demo_config", path, default_path);
    boost::thread queue_thread = boost::thread(boost::bind(&BehaviorDemo::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&BehaviorDemo::processThread, this));

    parseJointName(path);
}

BehaviorDemo::~BehaviorDemo()
{

}

void BehaviorDemo::parseJointName(string path)
{
    printf("parseJointName\n");
    module_joint_table_[head_tilt_name_] = "head_control_module";
    module_joint_table_[head_pan_name_] = "head_control_module";
}

void BehaviorDemo::callbackThread()
{
    head_control_pub_ = behavior_node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states_offset");

    ball_sub_ = behavior_node_->create_subscription<detector_msgs::msg::BallDetector>("/ball_lab_detector/result", std::bind(&BehaviorDemo::ballDetCallback, this, std::placeholders::_1));

    set_joint_module_client_ = behavior_client_node_->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");
    set_road_detector_client_ = behavior_client_node_->create_client<std_srvs::srv::SetBool>("/ball_lab_detector/enable");

    rclcpp::WallRate loop_rate(spin_rate_);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(behavior_node_);

        loop_rate.sleep();
    }
}

void BehaviorDemo::ballDetCallback(const detector_msgs::msg::BallDetector::SharedPtr msg)
{
    // printf("image:(%d, %d), ball:(%f, %f, %d)\n", msg->image_width, msg->image_height, msg->center_position.x, msg->center_position.y, msg->radius);
    ball_info_.image_width  = msg->image_width;
    ball_info_.image_height = msg->image_height;
    ball_info_.position_x   = msg->center_position.x;
    ball_info_.position_y   = msg->center_position.y;
    ball_info_.radius       = msg->radius;
    ball_info_.update       = true;
}

void BehaviorDemo::processThread()
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

void BehaviorDemo::process()
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

void BehaviorDemo::setModuleToDemo(const std::string module_name)
{
    if (enable_ == false)
        return;

    auto control_msg = std::make_shared<robotis_controller_msgs::msg::JointCtrlModule>();
    map<string, string>::iterator joint_iter = module_joint_table_.begin();
    
    for (; joint_iter != module_joint_table_.end(); joint_iter++)
    {
        if (joint_iter->second == module_name)
        {
            control_msg->joint_name.push_back(joint_iter->first);
            control_msg->module_name.push_back(module_name);
        }
    }

    if (control_msg->joint_name.size() == 0)
        return;

    callServiceSettingModule(control_msg);
}

void BehaviorDemo::callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr modules)
{
    auto set_joint_srv = std::make_shared<robotis_controller_msgs::srv::SetJointModule::Request>();
    set_joint_srv->joint_name = modules->joint_name;
    set_joint_srv->module_name = modules->module_name;

    printf("modules->joint_name.size(): %d\n", modules->joint_name.size());
    printf("set_joint_srv->joint_name.size(): %d\n", set_joint_srv->joint_name.size());
    while (!set_joint_module_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(behavior_client_node_->get_logger(), "client interrupted while waiting for service to appear.");
        }
            RCLCPP_INFO(behavior_client_node_->get_logger(), "waiting for service to appear...");
    }
    auto result_future = set_joint_module_client_->async_send_request(set_joint_srv);
    if (rclcpp::spin_until_future_complete(behavior_client_node_, result_future) != 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(behavior_client_node_->get_logger(), "Failed to set module");
        return;
    }
}

void BehaviorDemo::enableDet(bool enable)
{
    auto enable_detector = std::make_shared<std_srvs::srv::SetBool::Request>();

    enable_detector->data = enable;
    while (!set_road_detector_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(behavior_client_node_->get_logger(), "client interrupted while waiting for service to appear.");
        }
            RCLCPP_INFO(behavior_client_node_->get_logger(), "waiting for service to appear...");
    }
    auto result_future = set_road_detector_client_->async_send_request(enable_detector);
    if (rclcpp::spin_until_future_complete(behavior_client_node_, result_future) != 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(behavior_client_node_->get_logger(), "Failed to enable ball_detector");
        return;
    }
}

void BehaviorDemo::setEnable()
{
    enable_ = true;
    startDemo();
}

void BehaviorDemo::setDisable()
{
    stopDemo();
    enable_ = false;
}

void BehaviorDemo::startDemo()
{
    setModuleToDemo("head_control_module");
    enableDet(true);
}

void BehaviorDemo::stopDemo()
{
    setModuleToDemo("none");
    enableDet(false);
}
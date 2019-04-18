#include "behaviors/marathon.h"

using namespace robot_behavior;
using namespace std;

Marathon::Marathon()
    : spin_rate_(60)
{
    // init
    enable_ = false;
    road_info_ = new RoadInfo();
    road_info_->mark_exist = false;
    road_info_->road_exist = false;

    behavior_node_ = rclcpp::Node::make_shared("marathon_behavior");
    behavior_client_node_ = rclcpp::Node::make_shared("marathon_behavior_client");

    string default_path = " ";
    try
    {
        default_path = ament_index_cpp::get_package_share_directory("behaviors")+"/config/gui_config.yaml";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    string path = "";

    behavior_node_->get_parameter_or("demo_config", path, default_path);
    boost::thread queue_thread = boost::thread(boost::bind(&Marathon::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&Marathon::processThread, this));

    parseJointName(path);
}

Marathon::~Marathon()
{

}

void Marathon::parseJointName(string path)
{

}

void Marathon::callbackThread()
{

    road_sub_ = behavior_node_->create_subscription<detector_msgs::msg::RoadResult>("/road_detector/result", std::bind(&Marathon::roadDetCallback, this, std::placeholders::_1));

    set_joint_module_client_ = behavior_client_node_->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");
    rclcpp::WallRate loop_rate(spin_rate_);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(behavior_node_);

        loop_rate.sleep();
    }
}

void Marathon::roadDetCallback(const detector_msgs::msg::RoadResult::SharedPtr msg)
{
    road_info_->mark_exist   = msg->mark_exist;
    road_info_->road_exist   = msg->road_exist;
    road_info_->up_x         = msg->up_x;
    road_info_->up_y         = msg->up_y;
    road_info_->down_x       = msg->down_x;
    road_info_->down_y       = msg->down_y;
    road_info_->image_width  = msg->image_width;
    road_info_->image_height = msg->image_height;
    road_info_->direction    = Direction(msg->direction);
}

void Marathon::processThread()
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

void Marathon::process()
{
    if (!enable_)
        return;

}

void Marathon::setModuleToDemo(const std::string module_name)
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

void Marathon::callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr modules)
{
    auto set_joint_srv = std::make_shared<robotis_controller_msgs::srv::SetJointModule::Request>();
    set_joint_srv->joint_name = modules->joint_name;
    set_joint_srv->module_name = modules->module_name;

    printf("modules->joint_name.size(): %d\n", modules->joint_name.size());
    printf("set_joint_srv->joint_name.size(): %d\n", set_joint_srv->joint_name.size());
    auto result_future = set_joint_module_client_->async_send_request(set_joint_srv);
    if (rclcpp::spin_until_future_complete(behavior_client_node_, result_future) != 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(behavior_client_node_->get_logger(), "Failed to set module");
        return;
    }
}

void Marathon::setEnable()
{
    enable_ = true;
    startDemo();
}

void Marathon::setDisable()
{
    stopDemo();
    enable_ = false;
}

void Marathon::startDemo()
{
    setModuleToDemo("head_control_module");
}

void Marathon::stopDemo()
{
    
}
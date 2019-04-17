#include "cm_740_module/cm_740_module.h"

using namespace robotis_op;

Cm740Module::Cm740Module()
    : control_cycle_msec_(8),
      DEBUG_PRINT(false),
      ros_clock_(RCL_ROS_TIME),
      present_volt_(0.0)
{
  module_name_ = "cm_740_module";  // set unique module name
  module_node_ = rclcpp::Node::make_shared(module_name_);

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;

  result_["present_voltage"] = 0.0;
  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;

  last_msg_time_ = ros_clock_.now();
}

Cm740Module::~Cm740Module()
{
    queue_thread_.join();
}

void Cm740Module::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&Cm740Module::queueThread, this));
}

void Cm740Module::queueThread()
{
    /* publisher */
    status_msg_pub_ = module_node_->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status");
    imu_pub_ = module_node_->create_publisher<sensor_msgs::msg::Imu>("/robotis/cm_740/imu");
    button_pub_ = module_node_->create_publisher<std_msgs::msg::String>("/robotis/cm_740/button");

    rclcpp::WallRate loop_rate(1000.0/control_cycle_msec_);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(module_node_);
        loop_rate.sleep();
    }
}

void Cm740Module::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                          std::map<std::string, robotis_framework::Sensor *> sensors)
{
  // std::cout<<"cm-740 running...."<<std::endl;
  if (sensors["cm-740"] == NULL)
    return;

  uint16_t gyro_x = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_x"];
  uint16_t gyro_y = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_y"];
  uint16_t gyro_z = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_z"];

  uint16_t acc_x = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_x"];
  uint16_t acc_y = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_y"];
  uint16_t acc_z = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["cm-740"]->sensor_state_->bulk_read_table_["present_voltage"];

  result_["gyro_x"] = getGyroValue(gyro_x);
  result_["gyro_y"] = getGyroValue(gyro_y);
  result_["gyro_z"] = getGyroValue(gyro_z);

  RCLCPP_INFO(module_node_->get_logger(), "Gyro : %f, %f, %f", result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // align axis of Accelerometer to robot
  result_["acc_x"] = -getAccValue(acc_y);
  result_["acc_y"] = getAccValue(acc_x);
  result_["acc_z"] = -getAccValue(acc_z);

  RCLCPP_INFO(module_node_->get_logger(), "Acc : %f, %f, %f", result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  uint8_t button_flag = sensors["cm-740"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;

  handleButton("mode");
  handleButton("start");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  fusionIMU();
}

// -500 ~ 500dps, dps -> rps
double Cm740Module::getGyroValue(int raw_value)
{
  if(raw_value == 0)
    return 0;
  else
    return (raw_value - 512) * 500.0 * 2.0 / 1023 * DEGREE2RADIAN;
}

// -4.0 ~ 4.0g, 1g = 9.8 m/s^2
double Cm740Module::getAccValue(int raw_value)
{
  if(raw_value == 0)
    return 0;
  else
    return (raw_value - 512) * 4.0 * 2.0 / 1023;
}

void Cm740Module::fusionIMU()
{
  // fusion imu data
  imu_msg_.header.stamp = ros_clock_.now();
  imu_msg_.header.frame_id = "body_link";

  double filter_alpha = 0.4;

  //in rad/s
  long int _value = 0;
  int _arrd_length = 2;
  imu_msg_.angular_velocity.x = lowPassFilter(filter_alpha, result_["gyro_x"], imu_msg_.angular_velocity.x);
  imu_msg_.angular_velocity.y = lowPassFilter(filter_alpha, result_["gyro_y"], imu_msg_.angular_velocity.y);
  imu_msg_.angular_velocity.z = lowPassFilter(filter_alpha, result_["gyro_z"], imu_msg_.angular_velocity.z);

  //in m/s^2
  imu_msg_.linear_acceleration.x = lowPassFilter(filter_alpha, result_["acc_x"] * G_ACC,
                                                 imu_msg_.linear_acceleration.x);
  imu_msg_.linear_acceleration.y = lowPassFilter(filter_alpha, result_["acc_y"] * G_ACC,
                                                 imu_msg_.linear_acceleration.y);
  imu_msg_.linear_acceleration.z = lowPassFilter(filter_alpha, result_["acc_z"] * G_ACC,
                                                 imu_msg_.linear_acceleration.z);

  //Estimation of roll and pitch based on accelometer data, see http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
  double mui = 0.01;
  double sign = copysignf(1.0, result_["acc_z"]);
  double roll = atan2(result_["acc_y"],
                      sign * sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_x"] * result_["acc_x"]));
  double pitch = atan2(-result_["acc_x"],
                       sqrt(result_["acc_y"] * result_["acc_y"] + result_["acc_z"] * result_["acc_z"]));
  double yaw = 0.0;

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_->publish(imu_msg_);
}

void Cm740Module::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  // same state
  if (buttons_[button_key] == pushed)
  {
    if (pushed == true && buttons_[button_published] == false)
    {
      // check long press
      rclcpp::Duration button_duration = ros_clock_.now() - buttons_press_time_[button_name];
      if (button_duration.seconds() > 2.0)
      {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
    else if(button_name == "start" && (ros_clock_.now() - buttons_press_time_[button_name]).seconds() > 2.0)
    {
      if(buttons_[button_published])
        RCLCPP_INFO(module_node_->get_logger(), "start_long");
    }
  }
  else    // state is changed
  {
    buttons_[button_key] = pushed;

    if (pushed == true)
    {
      buttons_press_time_[button_name] = ros_clock_.now();
      buttons_[button_published] = false;
    }
    else
    {
      rclcpp::Duration button_duration = ros_clock_.now() - buttons_press_time_[button_name];

      if (button_duration.seconds() < 2)     // short press
      {
        publishButtonMsg(button_name);
      }
      else
        // long press
        ;

    }
  }
}

void Cm740Module::publishButtonMsg(const std::string &button_name)
{
  std_msgs::msg::String button_msg;
  button_msg.data = button_name;

  button_pub_->publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void Cm740Module::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
      (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    // check last published time
    rclcpp::Time now = ros_clock_.now();
    rclcpp::Duration dur = now - last_msg_time_;
    if (dur.seconds() < 1)
      return;

    last_msg_time_ = now;

    present_volt_ = previous_volt_;
    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
        (present_volt_ < 11.0 ?
            robotis_controller_msgs::msg::StatusMsg::STATUS_WARN : robotis_controller_msgs::msg::StatusMsg::STATUS_INFO),
        log_stream.str());
    RCLCPP_INFO(module_node_->get_logger(), "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  }
}

void Cm740Module::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = ros_clock_.now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

double Cm740Module::lowPassFilter(double alpha, double x_new, double x_old)
{
  return alpha * x_new + (1.0 - alpha) * x_old;
}
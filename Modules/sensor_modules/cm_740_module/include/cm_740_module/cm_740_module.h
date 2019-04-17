#ifndef CM_740_MODULE__CM_740_MODULE_H_
#define CM_740_MODULE__CM_740_MODULE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "cm_740_module/visibility_control.h"

namespace robotis_op
{

class Cm740Module : public robotis_framework::SensorModule, public robotis_framework::Singleton<Cm740Module>
{
public:
  Cm740Module();

  virtual ~Cm740Module();

    /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  const bool DEBUG_PRINT;
  const double G_ACC = 9.80665;

  void queueThread();

  double getGyroValue(int raw_value);
  double getAccValue(int raw_value);
  void fusionIMU();

  void handleButton(const std::string &button_name);
  void publishButtonMsg(const std::string &button_name);
  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  double lowPassFilter(double alpha, double x_new, double x_old);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  std::map<std::string, bool> buttons_;
  std::map<std::string, rclcpp::Time> buttons_press_time_;
  rclcpp::Time button_press_time_;
  rclcpp::Time last_msg_time_;
  rclcpp::Clock ros_clock_;
  rclcpp::Node::SharedPtr module_node_;
  double previous_volt_;
  double present_volt_;

  sensor_msgs::msg::Imu imu_msg_;

  /* subscriber & publisher */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
};

}  // namespace robtis_op

#endif  // CM_740_MODULE__CM_740_MODULE_H_

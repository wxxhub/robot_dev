#ifndef BALL_DETECTOR__BALL_DETECTOR_HPP_
#define BALL_DETECTOR__BALL_DETECTOR_HPP_
#include "rclcpp/rclcpp.hpp"
#include "ball_detector/visibility_control.h"

#include "sensor_msgs/msg/image.hpp"

namespace detector_module
{

class BallDetector : public rclcpp::Node
{
public:
  BallDetector();
  ~BallDetector();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

};

}  // namespace detector_module

#endif  // BALL_DETECTOR__BALL_DETECTOR_HPP_

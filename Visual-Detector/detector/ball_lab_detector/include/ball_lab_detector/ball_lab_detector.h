#ifndef BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_
#define BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_
#include <opencv2/highgui.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
// #include <boost/thread.hpp>
#include "rclcpp/rclcpp.hpp"

#include "ball_lab_detector/visibility_control.h"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "detector_msgs/msg/ball_detector.hpp"

namespace detector_module
{

enum Color
{
  RED,
  YELLOW
};

class BallLabDetector : public rclcpp::Node
{
public:
  BallLabDetector();
  ~BallLabDetector();

  // cv
  void process(cv::Mat image);
  void process();

  int detector(cv::Mat image);

  bool newImage();
  void showResult(cv::Mat image);
  void setShowResult(bool show_result);
  int encodingToMatType(const std::string & encoding);

private:
  cv::Mat input_image_;
  bool new_image_;
  bool show_result_;
  bool have_ball_;
  Color ball_color_;
  Color background_;

  int image_width_;
  int image_height_;

  double min_area_;
  int ball_x_;
  int ball_y_;
  int ball_radius_;

  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<detector_msgs::msg::BallDetector>::SharedPtr result_pub_;

  // ros2
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishResult();
};

}  // namespace ball_lab_detector

#endif  // BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_

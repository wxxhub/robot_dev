#ifndef BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_
#define BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_
#include <opencv2/highgui.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
// may throw PackageNotFoundError exception

// #include <boost/thread.hpp>
#include "rclcpp/rclcpp.hpp"

#include "ball_hsv_detector/visibility_control.h"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "ball_detector_msgs/msg/ball_detector.hpp"
#include "ball_hsv_detector/ball_detector_config.h"

namespace detector_module
{



enum Color
{
  RED,
  YELLOW
};

class BallHsvDetector : public rclcpp::Node
{
public:
  BallHsvDetector();
  ~BallHsvDetector();

  // cv
  void process(cv::Mat image);
  void process();

  int detector(cv::Mat image);
  void inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img);

  bool newImage();
  void showResult(cv::Mat image);
  void setShowResult(bool show_result);
  int encodingToMatType(const std::string & encoding);

  void resetParameter();

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

  HsvFilter hsv_filter1_;

  std::string default_setting_path_;

  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<ball_detector_msgs::msg::BallDetector>::SharedPtr result_pub_;

  // ros2
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishResult();
};

}  // namespace ball_hsv_detector

#endif  // BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_

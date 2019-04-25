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
#include "std_srvs/srv/set_bool.hpp"
#include "detector_msgs/msg/ball_detector.hpp"
#include "detector_msgs/srv/ball_set_params.hpp"
#include "ball_hsv_detector/ball_detector_config.h"

namespace detector_module
{



enum Color
{
  RED,
  YELLOW
};

class BallHsvDetector
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
  bool enable_;
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

  HsvFilter *hsv_filter1_;
  HsvFilter *backgroud_filter_;

  std::string default_setting_path_;

  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<detector_msgs::msg::BallDetector>::SharedPtr result_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_server_;
  rclcpp::Service<detector_msgs::srv::BallSetParams>::SharedPtr params_ser_;

  // ros2
  void noedThread();
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishResult();
  void enableServer(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setParamsServer(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<detector_msgs::srv::BallSetParams::Request> request,
                    const std::shared_ptr<detector_msgs::srv::BallSetParams::Response> response);
};

}  // namespace ball_hsv_detector

#endif  // BALL_LAB_DETECTOR__BALL_LAB_DETECTOR_HPP_

#ifndef ROAD_DETECTOR__ROAD_DETECTOR_H_
#define ROAD_DETECTOR__ROAD_DETECTOR_H_
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "road_detector/visibility_control.h"
#include "road_detector/arrow_detector.h"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "detector_msgs/msg/road_result.hpp"

namespace detector_module
{

enum Color
{
  RED,
  GREEN,
  YELLOW,
  BLUE,
  WHITE,
  BLACK
};

class ResultInfo
{
public:
	bool road_exist;
  bool mark_exist;
	cv::Point2f up_point, down_point;
	cv::Mat mark_image, angle_image;
	ArrowDirection direction;
	ResultInfo()
	{	
		road_exist = false;
    mark_exist = false;
		up_point = cv::Point2f(0,0);
		down_point = cv::Point2f(0,0);
		direction = DIRECT;
	}
};

class RoadDetector : public rclcpp::Node
{
public:
  RoadDetector();
  ~RoadDetector();

  // cv 
  void process(cv::Mat image);
  void process();
  int detector(cv::Mat image);
  bool getRoad(cv::Mat road_lab, cv::Point2f &up_point, cv::Point2f &down_point, float &road_angle);
  bool getMarkImage(cv::Mat mark_image, float &road_angle);
  bool newImage();
  void showResult(cv::Mat image);
  void setShowResult(bool show_result);
  int encodingToMatType(const std::string & encoding);

private:
  ResultInfo result_;
  cv::RotatedRect road_rect_;
  cv::Mat input_image_;

  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<detector_msgs::msg::RoadResult>::SharedPtr result_pub_;
  
  bool new_image_;
  bool show_result_;
  bool mark_detector_;
  bool enable_;
  const Color road_color;
  const bool wite_background_;

  int mark_rect_width_;
  int half_mark_rect_width_;

  int image_width_;
  int image_height_;

  ArrowDetector arrow_detector_;

  // ros2
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void enableServer(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void publishResult();

};

}  // namespace detector_module

#endif  // ROAD_DETECTOR__ROAD_DETECTOR_H_

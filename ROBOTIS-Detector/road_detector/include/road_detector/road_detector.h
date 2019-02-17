#ifndef ROAD_DETECTOR__ROAD_DETECTOR_HPP_
#define ROAD_DETECTOR__ROAD_DETECTOR_HPP_
#include <opencv2/highgui.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "road_detector/visibility_control.h"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace detector_module
{

enum Color
{
  RED,
  GREEN,
  YELLOW,
  BLUE,
  WHITE
};

class ResultInfo
{
public:
	bool bRoad;
	cv::Point2f up_point,down_point;

	bool bMark;
	cv::Mat imgmark,imgangle;
	int direction;
	ResultInfo()
	{	
		bRoad = false;
		up_point=cv::Point2f(0,0);
		down_point=cv::Point2f(0,0);
		bMark = false;

		direction = 0;
		
	}
};

const enum Color road_color = RED;

class RoadDetector : public rclcpp::Node
{
public:
  RoadDetector();

  virtual ~RoadDetector();

  // cv 
  void process(cv::Mat image);
  void process();
  int detector(cv::Mat image);
  bool getRoad(cv::Mat image, cv::Mat road_lab, cv::Point2f &up_point, cv::Point2f &down_point);
  void showResult(cv::Mat image);
  bool newImage();
  int encodingToMatType(const std::string & encoding);

private:
  ResultInfo result_;
  cv::RotatedRect road_rect_;
  cv::Mat input_image_;

  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;

  boost::thread queue_thread_;
  
  bool new_image_;

  // ros2
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void testCallback(const std_msgs::msg::String::SharedPtr msg);

};

}  // namespace detector_module

#endif  // ROAD_DETECTOR__ROAD_DETECTOR_HPP_

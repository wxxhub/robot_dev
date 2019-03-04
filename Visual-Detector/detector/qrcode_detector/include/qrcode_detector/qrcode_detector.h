#ifndef ROAD_DETECTOR__ROAD_DETECTOR_H_
#define ROAD_DETECTOR__ROAD_DETECTOR_H_
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
// #include <boost/thread.hpp>
#include "rclcpp/rclcpp.hpp"

#include "qrcode_detector/visibility_control.h"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "qrcode_detector_msgs/msg/qr_code_msg.hpp"
#include "qrcode_detector_msgs/msg/qr_code_list_msg.hpp"

#include "zbar.h"

namespace detector_module
{

struct QRCodeInfo
{
  std::string type_name;
  std::string data;
  int local_x;
  int local_y;
  int width;
  int height;
};


class QRCodeDetector : public rclcpp::Node
{
public:
  QRCodeDetector();

  virtual ~QRCodeDetector();

  // cv
  void process(cv::Mat image);
  void process();
  int detector(cv::Mat image);

  bool newImage();
  void showResult(cv::Mat image);
  int encodingToMatType(const std::string & encoding);

private:
  rclcpp::Node::SharedPtr detector_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<qrcode_detector_msgs::msg::QRCodeListMsg>::SharedPtr result_list_pub_;
  rclcpp::Publisher<qrcode_detector_msgs::msg::QRCodeMsg>::SharedPtr result_pub_;

  bool new_image_;

  cv::Mat input_image_;

  int image_width_;
  int image_height_;

  std::list<QRCodeInfo> qrcode_info_list_;

  //QRcode scanner
  zbar::ImageScanner scanner_;
  // ros2
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishResult();
};

}  // namespace qrcode_detector

#endif /* ROAD_DETECTOR__ROAD_DETECTOR_H_ */

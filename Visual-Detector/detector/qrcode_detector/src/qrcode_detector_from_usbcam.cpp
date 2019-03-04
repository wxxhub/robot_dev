#include <opencv2/highgui.hpp>

#include "qrcode_detector/qrcode_detector.h"

using namespace cv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto qrcode_detector = std::make_shared<detector_module::QRCodeDetector>();
  rclcpp::WallRate loop_rate(30);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(qrcode_detector);
    if (qrcode_detector->newImage())
    {
      qrcode_detector->process();
    }
    loop_rate.sleep();
  }
  // rclcpp::spin(qrcode_detector);
  // rclcpp::shutdown();

  printf("hello world qrcode_detector package\n");
  return 0;
}

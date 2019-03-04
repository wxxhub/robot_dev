#include <opencv2/highgui.hpp>

#include "qrcode_detector/qrcode_detector.h"

using namespace cv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  VideoCapture cap(0);
  rclcpp::WallRate loop_rate(30);

  if(!cap.isOpened())
	{
    printf("open video failed!\n");
		return -1;
	}
  
  Mat image;
  auto qrcode_detector = std::make_shared<detector_module::QRCodeDetector>();

  while(rclcpp::ok())
  {
    cap>>image;
    rclcpp::spin_some(qrcode_detector);
    qrcode_detector->process(image);
    loop_rate.sleep();
  }
  
  return 0;
}

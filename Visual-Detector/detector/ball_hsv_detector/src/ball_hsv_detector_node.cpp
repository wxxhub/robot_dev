#include <opencv2/highgui.hpp>

#include "ball_hsv_detector/ball_hsv_detector.h"
#include "timer/timer.hpp"

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
  Timer timer;
  auto ball_hsv_detector = std::make_shared<detector_module::BallHsvDetector>();
  ball_hsv_detector->setShowResult(true);
  while(rclcpp::ok())
  {
    cap>>image;
    rclcpp::spin_some(ball_hsv_detector);
    timer.reset();
    ball_hsv_detector->process(image);
    timer.stop();
    timer.show();
    loop_rate.sleep();
  }
  
  return 0;
}

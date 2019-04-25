#include <opencv2/highgui.hpp>

#include "ball_hsv_detector/ball_hsv_detector.h"
#include "timer/timer.hpp"

using namespace cv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  VideoCapture cap(0);

  if(!cap.isOpened())
	{
    printf("open video failed!\n");
		return -1;
	}
  
  Mat image;
  Timer timer;
  auto ball_hsv_detector = new detector_module::BallHsvDetector();
  ball_hsv_detector->setShowResult(true);
  while(rclcpp::ok())
  {
    cap>>image;
    timer.reset();
    ball_hsv_detector->process(image);
    timer.stop();
    // timer.show();
  }
  
  return 0;
}

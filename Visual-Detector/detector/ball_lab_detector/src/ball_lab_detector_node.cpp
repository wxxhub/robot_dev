#include <opencv2/highgui.hpp>

#include "ball_lab_detector/ball_lab_detector.h"
#include "timer/timer.hpp"

using namespace cv;
using namespace detector_module;

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
  BallLabDetector* ball_lab_detector = new BallLabDetector();  
  ball_lab_detector->setShowResult(true);
  while(rclcpp::ok())
  {
    cap>>image;
  #ifdef DEBUG_TIME
    timer.reset();
  #endif
    ball_lab_detector->process(image);
  #ifdef DEBUG_TIME
    timer.stop();
    timer.show();
  #endif
  }
  
  return 0;
}

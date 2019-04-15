#include <opencv2/highgui.hpp>

#include "ball_lab_detector/ball_lab_detector.h"
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
  auto ball_lab_detector = std::make_shared<detector_module::BallLabDetector>();
  ball_lab_detector->setShowResult(true);
  while(rclcpp::ok())
  {
    cap>>image;
    rclcpp::spin_some(ball_lab_detector);
  #ifdef DEBUG_TIME
    timer.reset();
  #endif
    ball_lab_detector->process(image);
  #ifdef DEBUG_TIME
    timer.stop();
    timer.show();
    timer.reset();
  #endif
    loop_rate.sleep();
  #ifdef DEBUG_TIME
    timer.stop();
    printf("sleep time: %fms\n", timer.elapsed());
  #endif
  }
  
  return 0;
}

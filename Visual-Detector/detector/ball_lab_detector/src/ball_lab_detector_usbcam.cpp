#include <opencv2/highgui.hpp>

#include "ball_lab_detector/ball_lab_detector.h"

using namespace cv;
using namespace detector_module;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::WallRate loop_rate(60);

  BallLabDetector* ball_lab_detector = new BallLabDetector();
  while (rclcpp::ok())
  {
    if (ball_lab_detector->newImage())
    {
      ball_lab_detector->process();
    }
  }

  printf("ball_lab_detector end\n");
  return 0;
}

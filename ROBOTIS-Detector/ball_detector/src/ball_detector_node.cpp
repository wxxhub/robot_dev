#include <opencv2/highgui.hpp>

#include "ball_detector/ball_detector.h"

using namespace cv;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ball_detector = std::make_shared<detector_module::BallDetector>();
  rclcpp::spin(ball_detector);
  rclcpp::shutdown();

  printf("hello world ball_detector package\n");
  return 0;
}

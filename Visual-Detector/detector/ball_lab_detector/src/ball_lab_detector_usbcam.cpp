#include <opencv2/highgui.hpp>

#include "ball_lab_detector/ball_lab_detector.h"

using namespace cv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::WallRate loop_rate(60);

  auto ball_lab_detector = std::make_shared<detector_module::BallLabDetector>();
  ball_lab_detector->setShowResult(true);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(ball_lab_detector);
    if (ball_lab_detector->newImage())
    {
      ball_lab_detector->process();
    }
    loop_rate.sleep();
  }

  printf("hello world ball_lab_detector package\n");
  return 0;
}

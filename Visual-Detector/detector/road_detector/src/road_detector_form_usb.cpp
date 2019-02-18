#include <opencv2/highgui.hpp>

#include "road_detector/road_detector.h"

using namespace cv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto road_detector = std::make_shared<detector_module::RoadDetector>();
  rclcpp::WallRate loop_rate(30);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(road_detector);
    if (road_detector->newImage())
    {
      road_detector->process();
    }
    loop_rate.sleep();
  }
  // rclcpp::spin(road_detector);
  // rclcpp::shutdown();

  printf("hello world road_detector package\n");
  return 0;
}

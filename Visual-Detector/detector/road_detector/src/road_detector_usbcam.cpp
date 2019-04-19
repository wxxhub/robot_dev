#include <opencv2/highgui.hpp>

#include "road_detector/road_detector.h"

using namespace cv;
using namespace detector_module;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RoadDetector* road_detector = new RoadDetector();
  road_detector->setShowResult(false);
  while (rclcpp::ok())
  {
    if (road_detector->newImage())
    {
      road_detector->process();
    }
  }

  printf("road_detector end\n");
  return 0;
}

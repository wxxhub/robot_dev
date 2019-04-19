#include <opencv2/highgui.hpp>

#include "road_detector/road_detector.h"

using namespace cv;
using namespace detector_module;

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
  RoadDetector* road_detector = new RoadDetector();
  road_detector->setShowResult(true);
  while(rclcpp::ok())
  {
    cap>>image;
    road_detector->process(image);
  }
  
  return 0;
}

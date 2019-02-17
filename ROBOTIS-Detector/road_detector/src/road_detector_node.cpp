#include <opencv2/highgui.hpp>

#include "road_detector/road_detector.h"

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
  detector_module::RoadDetector road_detector;

  while(rclcpp::ok())
  {
    cap>>image;
    road_detector.process(image);
    road_detector.showResult(image);
    // printf("cols: %d", image.cols);
  }
  
  return 0;
}

#ifndef ROAD_DETECTOR__ROAD_DETECTOR_HPP_
#define ROAD_DETECTOR__ROAD_DETECTOR_HPP_
#include <opencv2/highgui.hpp>

#include "road_detector/visibility_control.h"

namespace detector_module
{

enum Color
{
  RED,
  GREEN,
  YELLOW,
  BLUE,
  WHITE
};

class ResultInfo
{
public:
	bool bRoad;
	cv::Point2f up_point,down_point;

	bool bMark;
	cv::Mat imgmark,imgangle;
	int direction;
	ResultInfo()
	{	
		bRoad = false;
		up_point=cv::Point2f(0,0);
		down_point=cv::Point2f(0,0);
		bMark = false;

		direction = 0;
		
	}
};

const enum Color road_color = RED;

class RoadDetector
{
public:
  RoadDetector();

  virtual ~RoadDetector();

  void process(cv::Mat image);
  int detector(cv::Mat image);
  bool getRoad(cv::Mat image, cv::Mat road_lab, cv::Point2f &up_point, cv::Point2f &down_point);
  void showResult(cv::Mat image);

private:
  ResultInfo result;

  cv::RotatedRect road_rect;

};

}  // namespace detector_module

#endif  // ROAD_DETECTOR__ROAD_DETECTOR_HPP_

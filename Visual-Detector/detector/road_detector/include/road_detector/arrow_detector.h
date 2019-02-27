#ifndef _ARROW_DETECTER_H_
#define _ARROW_DETECTER_H_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace detector_module
{

enum ArrowDirection
{
  ERROR = -2,
  LEFT = -1,
  DIRECT = 0,
  RIGHT = 1,
};

class ArrowDetector
{
private:
  cv::Mat mark_image_;
  const int background_threshold_;

public:
  ArrowDetector();
  ArrowDirection getDirection(cv::Mat mark_image, bool wite_background);
  void getWatershedMark(cv::Mat mark_image, bool wite_background, cv::Mat &mark_watershed);
};

}   // namespace detector_module

#endif /*  _ARROW_DETECTER_H_ */
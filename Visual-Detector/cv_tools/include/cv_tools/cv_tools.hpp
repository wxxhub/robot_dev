#ifndef CV_TOOLS__CV_TOOLS_HPP_
#define CV_TOOLS__CV_TOOLS_HPP_

#include <opencv2/highgui.hpp>
#include "cv_tools/visibility_control.h"

namespace cv_tools
{
double getRectDegree(const cv::RotatedRect rect , cv::Point2f& up_point, cv::Point2f& down_point);
double calcLineDegree(const cv::Point2f& firstPt, const cv::Point2f& secondPt);



double getRectDegree(const cv::RotatedRect rect , cv::Point2f& up_point, cv::Point2f& down_point)
{
    double angle = 0.0f;
    cv::Point2f vertVect[4];
    rect.points(vertVect);

    const double firstLineLen = (vertVect[1].x - vertVect[0].x) * (vertVect[1].x - vertVect[0].x )
                              + (vertVect[1].y - vertVect[0].y) * (vertVect[1].y - vertVect[0].y);
    
    const double secondLineLen = (vertVect[2].x - vertVect[1].x) * (vertVect[2].x - vertVect[1].x)
		                       + (vertVect[2].y - vertVect[1].y) * (vertVect[2].y - vertVect[1].y);

    cv::Point2f p1,p2;
    if (firstLineLen > secondLineLen)
	{
		angle = calcLineDegree(vertVect[0], vertVect[1]);
		p1.x = (vertVect[0].x + vertVect[3].x)/2;
		p1.y = (vertVect[0].y + vertVect[3].y)/2;
		p2.x = (vertVect[1].x + vertVect[2].x)/2;
		p2.y = (vertVect[1].y + vertVect[2].y)/2;
	}
	else
	{
		angle = calcLineDegree(vertVect[2], vertVect[1]);
		p1.x = (vertVect[0].x + vertVect[1].x)/2;
		p1.y = (vertVect[0].y + vertVect[1].y)/2;
		p2.x = (vertVect[3].x + vertVect[2].x)/2;
		p2.y = (vertVect[3].y + vertVect[2].y)/2;
	}
	if(p2.y>p1.y)
	{
		down_point = p2;
		up_point = p1;
	}
	else
	{
		down_point  = p1;
		up_point = p2;
	}
    return angle;
}

double calcLineDegree(const cv::Point2f& firstPt, const cv::Point2f& secondPt)
{
	double curLineAngle = 0.0f;
	if (secondPt.x - firstPt.x != 0)
	{
		curLineAngle = atan(static_cast<double>(firstPt.y - secondPt.y) / static_cast<double>(secondPt.x - firstPt.x));
		if (curLineAngle < 0)
		{
			curLineAngle += CV_PI;
		}
	}
	else
	{
		curLineAngle = CV_PI / 2.0f; //90��
	}
	return curLineAngle*180.0f/CV_PI;
}
}  // namespace cv_tools

int encodingToMatType(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

#endif  // CV_TOOLS__CV_TOOLS_HPP_

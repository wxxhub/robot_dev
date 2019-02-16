#include <opencv2/opencv.hpp>

#include "road_detector/road_detector.h"
#include "cv_tools/cv_tools.hpp"

#define IMAGE_DEBUG
using namespace cv;

namespace detector_module
{

RoadDetector::RoadDetector()
{
}

RoadDetector::~RoadDetector()
{
}

void RoadDetector::process(Mat image)
{
    detector(image);
    imshow("road_detector",image);
    cvWaitKey(1);
}

int RoadDetector::detector(Mat image)
{
    Mat lab;
    std::vector<Mat> mv;
    Mat road_lab;
    Point2f up_point,down_point;
    cvtColor(image, lab, COLOR_BGR2Lab);
    imshow("road_detector lab", lab);

    split(lab, mv);

#ifdef IMAGE_DEBUG
    // imshow("mv[0]", mv[0]);
	// imshow("mv[1]", mv[1]);
	// imshow("mv[2]", mv[2]);
#endif /* IMAGE_DEBUG */

//     printf ("RED: %d\n", RED);
//     printf ("GREEN: %d\n", GREEN);
    switch (road_color)
    {
        case RED:
            road_lab = mv[1];
            break;

        case GREEN:
            road_lab = 255 - mv[1];
            break;
        
        case YELLOW:
            road_lab = mv[2];
            break;
        
        case BLUE:
            road_lab = 255 - mv[2];
            break;
        
        case WHITE:
            road_lab = mv[0];
            break;
    
        default:
            break;
    }

#ifdef IMAGE_DEBUG
    imshow("road_lab", road_lab);
#endif /* IMAGE_DEBUG */

    bool get_road_result = getRoad(image, road_lab, up_point, down_point);
    if  (get_road_result)
    {
       result.bRoad = true;
		result.up_point = up_point;
		result.down_point = down_point; 
    }
    else
    {
        result.bRoad = false;
		result.up_point = Point2f(0,0);
		result.down_point = Point2f(0,0);
    }
    
}

bool RoadDetector::getRoad(Mat image, Mat road_lab, Point2f &up_point,Point2f &down_point)
{
    Mat road_binary;
    threshold(road_lab, road_binary, 0, 255.0, CV_THRESH_BINARY | THRESH_OTSU);

#ifdef IMAGE_DEBUG
    imshow("road_binary", road_binary);
#endif /* IMAGE_DEBUG */

    //膨胀处理
    dilate(road_binary, road_binary, Mat());    

    //腐蚀处理
    erode(road_binary, road_binary, Mat());     
    erode(road_binary, road_binary, Mat());

#ifdef IMAGE_DEBUG
    imshow("after_road_binary", road_binary);
#endif /* IMAGE_DEBUG */

    std::vector<std::vector<Point> > contours;
    findContours(road_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    int max_rect_area = 0;
    Point min_left;
	Point max_right;
	Point max_down;
    for (size_t i = 0; i < contours.size(); i++)
    {
        std::vector<Point> vp = contours[i];
        double area = contourArea(contours[i]);

        RotatedRect rrect = minAreaRect(contours[i]);
        min_left.x = road_binary.cols/2;
		max_right.x=0;
		max_down.y=0;
        if (area > max_rect_area)
        {
            for (int j = 0; j < vp.size(); j++)
            {
                int left = vp[j].x;
                int right = left;
				int down = vp[j].y;
                if(left < min_left.x)
				{
					min_left = vp[j];
				}
				if(right > max_right.x)
				{
					max_right=vp[j];
				}
				if(down > max_down.y)
				{
					max_down=vp[j];
				}
            }
            road_rect = rrect;
            max_rect_area = area;
        }
    }

    float heighe_width = road_rect.size.height / road_rect.size.width;
    float angle = cv_tools::getRectDegree(road_rect, up_point, down_point);

/*  暂时没有效果
	Rect rr1 = rect.boundingRect();
	if(rr1.width>0.5*img.cols)
	{
		if(rr1.x <5)
		{
			uP = minleft;
			dP = maxdown;
		}
		if(rr1.x+rr1.width>img.cols-5)
		{
			uP = maxright;
			dP=  maxdown;
		}
	}

	float r2=maxrectarea /(rect.size.width*rect.size.height*1.0);
	if(rr1.y + rr1.height > img.rows-10 && rr1.height < rr1.width && r < 1.2 && r >0.8 && r2>0.5)
	{
		uP.x = rr1.x + rr1.width/2;
		uP.y = rr1.y;
		dP.x = uP.x;
		dP.y = rr1.y + rr1.height;
		angle-=90;
	}
*/

    // 避免突然闪烁全图识别

#ifdef IMAGE_DEBUG
    imshow("after_road_binary", road_binary);
#endif /* IMAGE_DEBUG */

    if (max_rect_area > 0.6 * image.cols * image.rows)
        return false;

    return true;
}

void RoadDetector::showResult(cv::Mat image)
{
    Mat show_img = image.clone();
    Point2f vertices[4];  
	road_rect.points(vertices);  
	for (int i = 0; i < 4; i++)  
		line(show_img, vertices[i], vertices[(i+1)%4], Scalar(255,255,0)); 

    line(show_img, result.up_point, result.down_point, Scalar(0,255,0),5);
    circle(show_img, result.up_point,13,Scalar(255,0,0),3);
	circle(show_img, result.down_point,13,Scalar(0,0,255),3);
    imshow("show_img", show_img);
}

}  // namespace detector_module

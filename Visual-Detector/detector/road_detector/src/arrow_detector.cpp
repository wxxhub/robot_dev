#include "road_detector/arrow_detector.h"

using namespace detector_module;
using namespace cv;

#define IMAGE_DEBUG

ArrowDetector::ArrowDetector()
    : background_threshold_(128)
{

}

ArrowDirection ArrowDetector::getDirection(Mat mark_image, bool wite_background)
{
    mark_image_ = mark_image.clone();
    Mat mark_watershed;
    std::vector<std::vector<Point>> contours_mark;
    getWatershedMark(mark_image_, wite_background, mark_watershed);
    
    // change background to wite
    Mat mark_binary = (mark_watershed!=background_threshold_);

    findContours(mark_binary, contours_mark, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    // //find max_area
    Rect max_mark_rect;
    int max_area = 0;
    for (size_t i = 0; i < contours_mark.size(); i++)
    {
        int now_area = contourArea(contours_mark[i]);
        if (now_area > max_area)
        {
            max_area = now_area;
            max_mark_rect = boundingRect(contours_mark[i]);
        }
        // Point p1 = Point(boundingRect(contours_mark[i]).x,boundingRect(contours_mark[i]).y);
        // Point p2 = Point(boundingRect(contours_mark[i]).x + boundingRect(contours_mark[i]).width,boundingRect(contours_mark[i]).y + boundingRect(contours_mark[i]).height);
        // rectangle(mark_image_,p1,p2,Scalar(255,0,0),1,8,0);
    }

    Mat mark;
    threshold(mark_watershed(max_mark_rect), mark, 130.0, 255.0, CV_THRESH_BINARY);
    
#ifdef IMAGE_DEBUG
    imshow("mark", mark);
    imshow("mark_image_2", mark_image_);
#endif /* IMAGE_DEBUG */

    Mat mark2 = mark.clone();
    std::vector<std::vector<Point>> contours(5);
	std::vector<std::vector<Point>> hierarchy;
    
    try{
        findContours(mark2, contours, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    }catch(cv::Exception){
        return ERROR;
    }

    std::vector<Rect> vec_rect;
    std::vector<int> vec_area;

    for (size_t i = 0; i < contours.size(); i++)
    {
        Rect rect1 = boundingRect(contours[i]);
        vec_rect.push_back(rect1);
        vec_area.push_back(contourArea(contours[i]));
    }

    Mat arrow_image;

    if (vec_rect.size() == 1)
    {
        mark(vec_rect[0]).copyTo(arrow_image);
    }
    else if(vec_rect.size() > 1)
    {
        Point standpt(mark.cols/2,mark.rows/2);
        int min_dist = 9999;
        Rect near_rect;
        for (int i=0; i<vec_rect.size(); i++)
        {
            Point pt = vec_rect[i].tl();
            pt.x = vec_rect[i].width/2;
            pt.y = vec_rect[i].height/2;
            int dist = abs(pt.x-standpt.x) + abs(pt.y-standpt.y);
            if (dist<min_dist)
            {
                near_rect = vec_rect[i];
                min_dist = dist;
            }
        }
        mark(near_rect).copyTo(arrow_image);
    }

#ifdef IMAGE_DEBUG
    imshow("arrow_image", arrow_image);
#endif /* IMAGE_DEBUG */

    if (arrow_image.cols*1.2 < arrow_image.rows)
    {
        return DIRECT;
    }
    else
    {
        /*
            |          |           |
            |left_rect | right_rect|
        */
        Rect left_rect(0, arrow_image.rows/2, arrow_image.cols/2, arrow_image.rows/2);
        Rect right_rect(arrow_image.cols/2, arrow_image.rows/2, arrow_image.cols/2, arrow_image.rows/2);

        Scalar sum_left = sum(arrow_image(left_rect));
        Scalar sum_right = sum(arrow_image(right_rect));

        if (sum_left[0] > sum_right[0])
            return RIGHT;
        else
            return LEFT;
    }
    
}

void ArrowDetector::getWatershedMark(Mat mark_image_, bool wite_background, Mat &mark_watershed)
{
    // color reversal
    if (wite_background)
    {
        std::vector<Mat> mv;

        split(mark_image_, mv);

        mv[0] = 255 - mv[0];
		mv[1] = 255 - mv[1];
		mv[2] = 255 - mv[2];
        merge(mv, mark_image_);
    }

    // get gray image
    Mat mark_gray;
    cvtColor(mark_image_, mark_gray, COLOR_BGRA2GRAY);
    GaussianBlur(mark_gray, mark_gray, Size(5,5), 2);

    double min_v,  max_v;
    int thread_high, thread_low;
    double lowr = 0.2, highr = 0.3;

    // find max gray value and min gray value
    minMaxLoc(mark_gray, &min_v, &max_v);

    // calculate split threshold
    thread_high = max_v - (max_v - min_v) * highr;
    thread_low  = min_v + (max_v - min_v) * lowr;

    Mat arrrow_gray, background_gray;

    // split threshold
    threshold(mark_gray, arrrow_gray, thread_high, 255, THRESH_BINARY);
    threshold(mark_gray, background_gray, thread_low, background_threshold_, THRESH_BINARY_INV);
    erode(arrrow_gray, arrrow_gray, Mat(), Point(-1, -1), 1);

    Mat mark_gray2 = arrrow_gray + background_gray;

    // watershed
    mark_gray2.convertTo(mark_gray2,CV_32S);
    watershed(mark_image_, mark_gray2);
    mark_gray2.convertTo(mark_watershed,CV_8U); 

#ifdef IMAGE_DEBUG
    imshow("mark_gray", mark_gray);
    imshow("arrrow_gray", arrrow_gray);
    imshow("background_gray", background_gray);
    imshow("mark_gray2", mark_gray2);
    imshow("watershed_marker", mark_watershed);
#endif /* IMAGE_DEBUG */
    return;
}
#include "road_detector/road_detector.h"
#include "cv_tools/cv_tools.hpp"

// #define IMAGE_DEBUG
using std::placeholders::_1;
using namespace cv;
using namespace detector_module;

RoadDetector::RoadDetector()
    : new_image_(false),
      Node("road_detector"),
      road_color(RED),
      show_result_(false),
      mark_detector_(true),
      wite_background_(true),
      mark_rect_width_(120),
      half_mark_rect_width_(mark_rect_width_/2),
      enable_(false)
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&RoadDetector::imageCallback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<detector_msgs::msg::RoadResult>("/road_detector/result");

    auto enable_server = this->create_service<std_srvs::srv::SetBool>("/road_detector/enable", std::bind(&RoadDetector::enableServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

RoadDetector::~RoadDetector()
{
    
}

void RoadDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static bool first_image = true;
    // printf("new image test\n");
    Mat frame(msg->height, msg->width, encodingToMatType(msg->encoding),
              const_cast<unsigned char *>(msg->data.data()), msg->step);
    
    if (first_image)
    {
        image_width_ = msg->width;
        image_height_ = msg->height;
        first_image = false;
    }

    input_image_ = frame.clone();
    new_image_ = true;
}

void RoadDetector::enableServer(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    enable_ = request->data;
    response->success = true;
}

void RoadDetector::process()
{
    if (!enable_)
        return;

    detector(input_image_);
    publishResult();
    showResult(input_image_);
}

void RoadDetector::process(Mat image)
{
    if (!enable_)
        return;

    static bool first_image = true;
    if (first_image)
    {
        image_width_ = image.cols;
        image_height_ = image.rows;
        first_image = false;
    }
    input_image_ = image.clone();
    detector(input_image_);
    publishResult();
    showResult(image);
}

int RoadDetector::detector(Mat image)
{
    Mat lab;
    std::vector<Mat> mv;
    Mat road_lab;
    Point2f up_point,down_point;
    cvtColor(image, lab, COLOR_BGR2Lab);
    // imshow("road_detector lab", lab);

    split(lab, mv);

#ifdef IMAGE_DEBUG
    // imshow("mv[0]", mv[0]);
	// imshow("mv[1]", mv[1]);
	// imshow("mv[2]", mv[2]);
#endif /* IMAGE_DEBUG */

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
    try
    {
        imshow("road_lab", road_lab);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
#endif /* IMAGE_DEBUG */

    float road_angle = 0;
    bool get_road_result = getRoad(road_lab, up_point, down_point, road_angle);
    if  (get_road_result)
    {
        result_.road_exist = true;
		result_.up_point = up_point;
		result_.down_point = down_point; 
    }
    else
    {
        result_.road_exist = false;
		result_.up_point = Point2f(0,0);
		result_.down_point = Point2f(0,0);
    }

    if (!mark_detector_)
        return 1;
    
    Mat mark_image;

    if (!getMarkImage(mark_image, road_angle))
        return 2;
    
    result_.mark_image = mark_image.clone();
    ArrowDirection direction_result = arrow_detector_.getDirection(mark_image, wite_background_);
    if (direction_result != ERROR)
        result_.direction = direction_result;
    else
        result_.direction = DIRECT;

/* arrow test code
    Mat mark_image = input_image_.clone();
    ArrowDirection direction_result = arrow_detector_.getDirection(mark_image, wite_background_);
    if (direction_result != ERROR)
        result_.direction = direction_result;
    else
        result_.direction = DIRECT;
*/ 
}

bool RoadDetector::getRoad(Mat road_lab, Point2f &up_point, Point2f &down_point, float &road_angle)
{
    Mat road_binary;
    threshold(road_lab, road_binary, 0, 255.0, CV_THRESH_BINARY | THRESH_OTSU);

#ifdef IMAGE_DEBUG
    try
    {
        // imshow("road_binary", road_binary);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
#endif /* IMAGE_DEBUG */

    //膨胀处理
    dilate(road_binary, road_binary, Mat());    

    //腐蚀处理
    erode(road_binary, road_binary, Mat());     
    erode(road_binary, road_binary, Mat());

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
            road_rect_ = rrect;
            max_rect_area = area;
        }
    }

    float heighe_width = road_rect_.size.height / road_rect_.size.width;
    road_angle = cv_tools::getRectDegree(road_rect_, up_point, down_point);

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
    try
    {
        imshow("after_road_binary", road_binary);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
#endif /* IMAGE_DEBUG */

    if (max_rect_area > 0.6 * road_lab.cols * road_lab.rows)
        return false;

    return true;
}

bool RoadDetector::getMarkImage(cv::Mat mark_image, float &road_angle)
{
    //  如果不存在路
    if (!mark_detector_ || !result_.road_exist)
        return false;

    // 是否到路线端头
    if (result_.up_point.x < image_width_ * 0.1 || 
        result_.up_point.x > image_width_ * 0.9 ||
        result_.up_point.y < image_height_ * 0.3)
    {
        return false;
    }

    // 路标区域超出图片范围
    if (result_.up_point.x - half_mark_rect_width_ <= 0 || result_.up_point.x + half_mark_rect_width_ >= image_width_)
    {
        return false;
    }

    Mat rotated_image, rotated_mat;

    // 计算路标需要旋转的角度
    road_angle -= 90;
    road_angle = -road_angle;
    //获取旋转矩阵
    rotated_mat = getRotationMatrix2D(Point(result_.up_point.x, result_.up_point.y), road_angle, 1.0);

    /// 旋转已扭曲图像
    warpAffine(input_image_, rotated_image, rotated_mat, input_image_.size(), INTER_CUBIC, 1);

    //路线上部的一片区域
    //---- mark_rect_width_

    //    ----
    //    |   |     mark_rect   标记所在区域 ; * result.up_point 
    //    --*--
    //      |
    //      |
    //      @       @ result.down_point 

    Rect mark_rect = Rect(result_.up_point.x - half_mark_rect_width_, result_.up_point.y - mark_rect_width_, mark_rect_width_, mark_rect_width_);
    mark_rect &= Rect(0, 0, rotated_image.cols, rotated_image.rows);
    rotated_image(mark_rect).copyTo(mark_image);

#ifdef IMAGE_DEBUG
    try
    {
        // imshow("rotated_image", rotated_image);
        imshow("mark_image", mark_image);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
#endif /* IMAGE_DEBUG */
    result_.angle_image = rotated_mat.clone();
    return true;
}

void RoadDetector::showResult(cv::Mat image)
{
    if (show_result_)
    {
        Mat result_image = image.clone();
        Point2f vertices[4];  
        road_rect_.points(vertices);  
        for (int i = 0; i < 4; i++)  
            line(result_image, vertices[i], vertices[(i+1)%4], Scalar(255,255,0)); 

        line(result_image, result_.up_point, result_.down_point, Scalar(0,255,0),5);
        circle(result_image, result_.up_point,13,Scalar(255,0,0),3);
        circle(result_image, result_.down_point,13,Scalar(0,0,255),3);

        try
        {
            imshow("Roade_result", result_image);
            cvWaitKey(1);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

void RoadDetector::setShowResult(bool show_result)
{
  show_result_ = show_result;
}

bool RoadDetector::newImage()
{
    if (new_image_)
    {
        new_image_ = false;
        return true;
    }
    else
    {
        return false;
    }
    
    return new_image_;
}

void RoadDetector::publishResult()
{
    auto message = detector_msgs::msg::RoadResult();
    message.road_exist = result_.road_exist;
    message.mark_exist = result_.mark_exist;
    message.up_x = result_.up_point.x;
    message.up_y = result_.up_point.y;
    message.down_x = result_.down_point.x;
    message.down_y = result_.down_point.y;
    message.direction = result_.direction;
    message.image_width = image_width_;
    message.image_height = image_height_;
    result_pub_->publish(message);
}

int RoadDetector::encodingToMatType(const std::string & encoding)
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


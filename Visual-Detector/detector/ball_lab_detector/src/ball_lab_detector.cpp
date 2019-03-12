#include "ball_lab_detector/ball_lab_detector.h"

using namespace cv;
using namespace std;
using namespace detector_module;

#define IMAGE_DEBUG

BallLabDetector::BallLabDetector()
    : new_image_(false),
      Node("ball_lab_detector"),
      show_result_(false),
      ball_color_(RED),
      min_area_(200.0)
{
    /* sublisher */
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&BallLabDetector::imageCallback, this, std::placeholders::_1));

    /* publisger */
    result_pub_ = this->create_publisher<ball_detector_msgs::msg::BallDetector>("/ball_lab_detector/result");
}

BallLabDetector::~BallLabDetector()
{

}

void BallLabDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static bool first_image = true;
    Mat frame(msg->height, msg->width, encodingToMatType(msg->encoding),
              const_cast<unsigned char*>(msg->data.data()), msg->step);

    // get image_size
    if (first_image)
    {
        image_width_ = msg->width;
        image_height_ = msg->height;
        first_image = false;
    }

    input_image_ = frame.clone();
    new_image_ = true;
}

void BallLabDetector::process(cv::Mat image)
{
    static bool first_image = true;
    if (first_image)
    {
        image_width_ = image.cols;
        image_height_ = image.rows;
        first_image = false;
    }
    input_image_ = image.clone();

    int detector_result = detector(image);
    if (detector_result != 0)
        publishResult();

    showResult(image);
}

void BallLabDetector::process()
{
    int detector_result = detector(input_image_);
    if (detector_result != 0)
        publishResult();

    showResult(input_image_);
}

int BallLabDetector::detector(cv::Mat image)
{
    Mat detector_image = image.clone();
    Mat lab_image, ball_lab, ball_binary;
    Mat background_lab, background_binary;

    std::vector<Mat> mv;

    cvtColor(detector_image, lab_image, COLOR_BGR2Lab);

    split(lab_image, mv);

#ifdef IMAGE_DEBUG
    // try
    // {
    //     imshow("lab_image", lab_image);
    //     imshow("mv[0]", mv[0]);
    //     imshow("mv[1]", mv[1]);
    //     imshow("mv[2]", mv[2]);
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
#endif /* IMAGE_DEBUG */   

    switch (ball_color_)
    {                                       // 去掉蓝黄             //   减少白色干扰
        case RED:
            ball_lab = 3 * (mv[1] - 127) - 2 * abs(mv[2] -127) -  0.1 * mv[0];
            break;
        
        case YELLOW:
            ball_lab = 2 * (mv[2] - 127) - 2 * abs(mv[1] -127);
            break;
    
        default:
            printf("Please set ball_color_!\n");
            exit(-1);
            break;
    }

#ifdef IMAGE_DEBUG
    // try
    // {
    //     imshow("ball_lab", ball_lab);
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
#endif /* IMAGE_DEBUG */

    //滤波
    GaussianBlur(ball_lab, ball_lab, Size(9, 9), 2, 2 );
    // equalizeHist(ball_lab, ball_lab);
    threshold(ball_lab, ball_binary, 5, 255.0, CV_THRESH_BINARY);
     //膨胀处理
    dilate(ball_binary, ball_binary, Mat());

    //腐蚀处理
    erode(ball_binary, ball_binary, Mat());
    erode(ball_binary, ball_binary, Mat());

    // 找出边缘
    vector<vector<Point>> contours;
    findContours(ball_lab, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    //选择最大区域
    double max_area = 0;
    vector<Point> max_contour;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            max_contour = contours[i];
        }
    }

    // printf ("max_area: %f\n", max_area);
    Rect max_rect = boundingRect(max_contour);

#ifdef IMAGE_DEBUG
    try
    {
        // rectangle(detector_image, max_rect, Scalar(0, 255, 0));
        // imshow("detector_image2", detector_image);
        imshow("ball_lab_circle", ball_lab);
        imshow("ball_binary", ball_binary);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    cvWaitKey(10);
#endif /* IMAGE_DEBUG */

    ball_x_ = 0;
    ball_y_ = 0;
    ball_radius_ = 0;
    if (max_area>min_area_)
    {
        double half_width = max_rect.width * 0.5;
        double half_height = max_rect.height * 0.5;
        ball_radius_ = max(half_width, half_height);
        ball_x_ = max_rect.x + half_width;
        ball_y_ = max_rect.y + half_height;
        return 1;
    }
    return 0;
}

bool BallLabDetector::newImage()
{
    if (new_image_)
    {
        new_image_ = false;
        return true;
    }

    return new_image_;
}

void BallLabDetector::showResult(cv::Mat image)
{
    if (!show_result_)
        return;
    
    try
    {
        circle(image, Point(ball_x_, ball_y_), ball_radius_, Scalar(0, 255, 0), 2);
        imshow("result_image", image);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void BallLabDetector::setShowResult(bool show_result)
{
    show_result_ = show_result;
}

void BallLabDetector::publishResult()
{
    auto message = ball_detector_msgs::msg::BallDetector();
    message.center_position.x = ball_x_;
    message.center_position.y = ball_y_;
    message.radius = ball_radius_;
    message.image_width = image_width_;
    message.image_height = image_height_;

    result_pub_->publish(message);
}

int BallLabDetector::encodingToMatType(const std::string & encoding)
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

#include "ball_detector/ball_detector.h"

using std::placeholders::_1;
using namespace detector_module;

BallDetector::BallDetector()
  : Node("ball_detector")
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&BallDetector::imageCallback, this, std::placeholders::_1));
}

BallDetector::~BallDetector()
{
}

void BallDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    printf("new image\n");
}

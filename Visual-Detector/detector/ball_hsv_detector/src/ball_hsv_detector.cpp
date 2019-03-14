#include "ball_hsv_detector/ball_hsv_detector.h"

using namespace cv;
using namespace std;
using namespace detector_module;

#define IMAGE_DEBUG

BallHsvDetector::BallHsvDetector()
    : new_image_(false),
      Node("ball_hsv_detector"),
      show_result_(false),
      ball_color_(RED),
      min_area_(200.0)
{
    /* sublisher */
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&BallHsvDetector::imageCallback, this, std::placeholders::_1));

    /* publisger */
    result_pub_ = this->create_publisher<ball_detector_msgs::msg::BallDetector>("/ball_hsv_detector/result");
    string package_share_directory = ament_index_cpp::get_package_share_directory("ball_hsv_detector");
    cout<<"package_share_directory test: "<<package_share_directory<<endl;
}

BallHsvDetector::~BallHsvDetector()
{

}

void BallHsvDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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

void BallHsvDetector::process(cv::Mat image)
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

void BallHsvDetector::process()
{
    int detector_result = detector(input_image_);
    if (detector_result != 0)
        publishResult();

    showResult(input_image_);
}

int BallHsvDetector::detector(cv::Mat image)
{

    return 0;
}

bool BallHsvDetector::newImage()
{
    if (new_image_)
    {
        new_image_ = false;
        return true;
    }

    return new_image_;
}

void BallHsvDetector::resetParameter()
{
//   YAML::Node doc;

//   try
//   {
//     // load yaml
//     doc = YAML::LoadFile(default_setting_path_.c_str());

//     // parse
//     params_config_.gaussian_blur_size = doc["gaussian_blur_size"].as<int>();
//     params_config_.gaussian_blur_sigma = doc["gaussian_blur_sigma"].as<double>();
//     params_config_.canny_edge_th = doc["canny_edge_th"].as<double>();
//     params_config_.hough_accum_resolution = doc["hough_accum_resolution"].as<double>();
//     params_config_.min_circle_dist = doc["min_circle_dist"].as<double>();
//     params_config_.hough_accum_th = doc["hough_accum_th"].as<double>();
//     params_config_.min_radius = doc["min_radius"].as<int>();
//     params_config_.max_radius = doc["max_radius"].as<int>();
//     params_config_.filter_threshold.h_min = doc["filter_h_min"].as<int>();
//     params_config_.filter_threshold.h_max = doc["filter_h_max"].as<int>();
//     params_config_.filter_threshold.s_min = doc["filter_s_min"].as<int>();
//     params_config_.filter_threshold.s_max = doc["filter_s_max"].as<int>();
//     params_config_.filter_threshold.v_min = doc["filter_v_min"].as<int>();
//     params_config_.filter_threshold.v_max = doc["filter_v_max"].as<int>();
//     params_config_.use_second_filter = doc["use_second_filter"].as<bool>();
//     params_config_.filter2_threshold.h_min = doc["filter2_h_min"].as<int>();
//     params_config_.filter2_threshold.h_max = doc["filter2_h_max"].as<int>();
//     params_config_.filter2_threshold.s_min = doc["filter2_s_min"].as<int>();
//     params_config_.filter2_threshold.s_max = doc["filter2_s_max"].as<int>();
//     params_config_.filter2_threshold.v_min = doc["filter2_v_min"].as<int>();
//     params_config_.filter2_threshold.v_max = doc["filter2_v_max"].as<int>();
//     params_config_.ellipse_size = doc["ellipse_size"].as<int>();
//     params_config_.debug = doc["filter_debug"].as<bool>();

//     // gaussian_blur has to be odd number.
//     if (params_config_.gaussian_blur_size % 2 == 0)
//       params_config_.gaussian_blur_size -= 1;
//     if (params_config_.gaussian_blur_size <= 0)
//       params_config_.gaussian_blur_size = 1;

//     printConfig();
//     saveConfig();

//     publishParam();
//   } catch (const std::exception& e)
//   {
//     ROS_ERROR_STREAM("Failed to Get default parameters : " << default_setting_path_);
//     return;
//   }
}

void BallHsvDetector::showResult(cv::Mat image)
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

void BallHsvDetector::setShowResult(bool show_result)
{
    show_result_ = show_result;
}

void BallHsvDetector::publishResult()
{
    auto message = ball_detector_msgs::msg::BallDetector();
    message.center_position.x = ball_x_;
    message.center_position.y = ball_y_;
    message.radius = ball_radius_;
    message.image_width = image_width_;
    message.image_height = image_height_;

    result_pub_->publish(message);
}

int BallHsvDetector::encodingToMatType(const std::string & encoding)
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

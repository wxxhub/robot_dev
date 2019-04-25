#include "ball_hsv_detector/ball_hsv_detector.h"

using namespace cv;
using namespace std;
using namespace detector_module;

#define IMAGE_DEBUG

BallHsvDetector::BallHsvDetector()
    : new_image_(false),
      show_result_(false),
      ball_color_(RED),
      min_area_(200.0),
	    ball_x_(0),
	    ball_y_(0)
{
    /* init data */
    hsv_filter1_ = new HsvFilter();
    backgroud_filter_ = new HsvFilter();

    /* sublisher */
    image_sub_ = detector_node_->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&BallHsvDetector::imageCallback, this, std::placeholders::_1));

    /* publisger */
    result_pub_ = detector_node_->create_publisher<detector_msgs::msg::BallDetector>("/ball_hsv_detector/result");

    /* server */
    enable_server_ = detector_node_->create_service<std_srvs::srv::SetBool>("/road_detector/enable", std::bind(&BallHsvDetector::enableServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // params_ser_ = detector_node_->create_service<detector_msgs::srv::BallSetParams>("/ball_hsv_detector/set_params", std::bind(&BallHsvDetector::setParamsServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    try
    {
      default_setting_path_ = ament_index_cpp::get_package_share_directory("ball_hsv_detector") + "/config/ball_detector_params.yaml";
		  resetParameter();
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

BallHsvDetector::~BallHsvDetector()
{
	
}

void BallHsvDetector::noedThread()
{
    rclcpp::WallRate loop_rate(60);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(detector_node_);
        loop_rate.sleep();
    }
    
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

void BallHsvDetector::enableServer(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    enable_ = request->data;
    response->success = true;
}

void BallHsvDetector::setParamsServer(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<detector_msgs::srv::BallSetParams::Request> request,
                                      const std::shared_ptr<detector_msgs::srv::BallSetParams::Response> response)
{
    hsv_filter1_->h_min = request->params.filter_h_min;
    hsv_filter1_->h_max = request->params.filter_h_max;
    hsv_filter1_->s_min = request->params.filter_s_min;
    hsv_filter1_->s_max = request->params.filter_s_max;
    hsv_filter1_->v_min = request->params.filter_v_min;
    hsv_filter1_->v_max = request->params.filter_v_max;
}

void BallHsvDetector::process(Mat image)
{
    static bool first_image = true;
    if (first_image)
    {
        image_width_ = image.cols;
        image_height_ = image.rows;
        first_image = false;
    }
    input_image_ = image.clone();

    int detector_result = detector(input_image_);
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

int BallHsvDetector::detector(Mat image)
{
	Mat detect_image = image.clone();
	Mat hsv_image, filtered_image;
	cvtColor(detect_image,  hsv_image, COLOR_RGB2HSV);

	inRangeHsv(hsv_image, *hsv_filter1_, filtered_image);

#ifdef IMAGE_DEBUG
    try
    {
		imshow("hsv_image", hsv_image);
        imshow("filtered_image", filtered_image);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
#endif
    return 0;
}

void BallHsvDetector::inRangeHsv(const Mat &input_img, const HsvFilter &filter_value, Mat &output_img)
{
	// 0-360 -> 0-180
	int scaled_hue_min = static_cast<int>(filter_value.h_min * 0.5);
	int scaled_hue_max = static_cast<int>(filter_value.h_max * 0.5);

	if (scaled_hue_min <= scaled_hue_max)
	{
		Scalar min_value = Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
		Scalar max_value = Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);

		inRange(input_img, min_value, max_value, output_img);
	}
	else
	{
		Mat lower_hue_range, upper_hue_range;
		Scalar min_value, max_value;

		min_value = Scalar(0, filter_value.s_min, filter_value.v_min, 0);
		max_value = Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);
		inRange(input_img, min_value, max_value, lower_hue_range);

		min_value = Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
		max_value = Scalar(179, filter_value.s_max, filter_value.v_max, 0);
		inRange(input_img, min_value, max_value, upper_hue_range);

		bitwise_or(lower_hue_range, upper_hue_range, output_img);
	}
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
	YAML::Node doc;

	try
	{
		// load yaml
		doc = YAML::LoadFile(default_setting_path_.c_str());

		hsv_filter1_->h_min = doc["filter_h_min"].as<int>();
		hsv_filter1_->h_max = doc["filter_h_max"].as<int>();
		hsv_filter1_->s_min = doc["filter_s_min"].as<int>();
		hsv_filter1_->s_max = doc["filter_s_max"].as<int>();
		hsv_filter1_->v_min = doc["filter_v_min"].as<int>();
		hsv_filter1_->v_max = doc["filter_v_max"].as<int>();
		hsv_filter1_->print();

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
	} catch (const std::exception& e)
	{
		RCLCPP_ERROR(detector_node_->get_logger(), "Failed to open config file: %s", default_setting_path_);
		return;
	}
}

void BallHsvDetector::showResult(Mat image)
{
    if (!show_result_)
        return;
    
    try
    {
        // circle(image, Point(ball_x_, ball_y_), ball_radius_, Scalar(0, 255, 0), 2);
        imshow("result_image", image);
		cvWaitKey(1);
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
    auto message = detector_msgs::msg::BallDetector();
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

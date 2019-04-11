
#include "qrcode_detector/qrcode_detector.h"

using namespace cv;
using namespace zbar;
using namespace detector_module;

QRCodeDetector::QRCodeDetector()
    : new_image_(false),
      Node("QRcode_detector"),
      show_result_(false)
{
  /* sublisher */
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam_pub/image0", std::bind(&QRCodeDetector::imageCallback, this, std::placeholders::_1));

  /* publisher */
  result_list_pub_ = this->create_publisher<detector_msgs::msg::QRCodeListMsg>("/qrcode_detector/list_result");
  result_pub_ = this->create_publisher<detector_msgs::msg::QRCodeMsg>("/qrcode_detector/result");
  
  scanner_.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
}

QRCodeDetector::~QRCodeDetector()
{

}

void QRCodeDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static bool first_image = true;
  Mat frame(msg->height, msg->width, encodingToMatType(msg->encoding),
            const_cast<unsigned char *>(msg->data.data()), msg->step);
  
  // get image size
  if (first_image)
  {
    image_width_ = msg->width;
    image_height_ = msg->height;
    first_image = false;
  }

  input_image_ = frame.clone();
  new_image_ = true;
}

void QRCodeDetector::process(Mat image)
{
  // get image size
  static bool first_image = true;
  if (first_image)
  {
    image_width_ = image.cols;
    image_height_ = image.rows;
    first_image = false;
  }

  int detector_result = detector(image);
  if (detector_result != 0)
    publishResult();

  showResult(image);
}

void QRCodeDetector::process()
{
  int detector_result = detector(input_image_);
  if (detector_result != 0)
    publishResult();

  showResult(input_image_);
}

int QRCodeDetector::detector(Mat image)
{
  int detector_result = 0;
  qrcode_info_list_.clear();

  Mat image_gray;
  cvtColor(image, image_gray, CV_RGB2GRAY);

  uchar *raw = (uchar *)image_gray.data;
  Image image_zbar(image.cols, image.rows, "Y800", raw, image.cols*image.rows);
  scanner_.scan(image_zbar);

  Image::SymbolIterator symbol_iter = image_zbar.symbol_begin();

  // get result
  for (; symbol_iter != image_zbar.symbol_end(); ++symbol_iter)
  {
    detector_result = 1;
    QRCodeInfo qrcode_info;
    qrcode_info.type_name = symbol_iter->get_type_name();
    qrcode_info.data = symbol_iter->get_data();
    for (int i = 0; i < 4; i++)
    {
      qrcode_info.vertex_position[i].x = symbol_iter->get_location_x(i);
      qrcode_info.vertex_position[i].y = symbol_iter->get_location_y(i);
    }
    qrcode_info_list_.push_back(qrcode_info);
  }
  return detector_result;
}

bool QRCodeDetector::newImage()
{
  if (new_image_)
  {
    new_image_ = false;
    return true;
  }
  
  return new_image_;
}

void QRCodeDetector::showResult(Mat image)
{
  if (!show_result_)
    return;
  Mat result_image = image.clone();
  std::list<QRCodeInfo>::iterator list_iter = qrcode_info_list_.begin();
  for (; list_iter != qrcode_info_list_.end(); ++list_iter)
  {
    // draw result
    rectangle(result_image, list_iter->vertex_position[0], list_iter->vertex_position[2], Scalar(0,255,0), 1);
  }

  try
  {
    imshow("QRcode_result", result_image);
    cvWaitKey(1);
  } 
  catch (const std::exception& e)
  {
    std::cerr << "QRcode_result: " << e.what() << '\n';
  }
  
}

void QRCodeDetector::setShowResult(bool show_result)
{
  show_result_ = show_result;
}

void QRCodeDetector::publishResult()
{
  detector_msgs::msg::QRCodeListMsg msg_list;
  std::list<QRCodeInfo>::iterator list_iter = qrcode_info_list_.begin();
  for (; list_iter != qrcode_info_list_.end(); ++list_iter)
  {
    detector_msgs::msg::QRCodeMsg info_msg;
    info_msg.type_name     = list_iter->type_name;
    info_msg.data          = list_iter->data;
    info_msg.image_width   = image_width_;
    info_msg.image_height  = image_height_;
    for (int i = 0; i < 4; i++)
    {
      info_msg.vertex_position[i].x = list_iter->vertex_position[i].x;
      info_msg.vertex_position[i].y = list_iter->vertex_position[i].y;
    }
    msg_list.message_list.push_back(info_msg);
    result_pub_->publish(info_msg);
  }
  result_list_pub_->publish(msg_list);
}

int QRCodeDetector::encodingToMatType(const std::string & encoding)
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
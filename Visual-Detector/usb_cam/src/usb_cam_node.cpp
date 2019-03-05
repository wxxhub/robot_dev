#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <boost/thread.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

#define IMAGE_PUB_PTR std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image_<std::allocator<void> >, std::allocator<void> > >

const int NO_DEVICE_NUM = -1;
const std::string DEVICE_0_NAME = "device_0";
const std::string DEVICE_1_NAME = "device_1";
const std::string DEVICE_2_NAME = "device_2";
const std::string DEVICE_3_NAME = "device_3";
const std::string DEVICE_4_NAME = "device_4";

boost::mutex  open_video_mutex_;

struct DeviceInfo
{
  int device_num = -1;
  IMAGE_PUB_PTR image_pub;
};

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

void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
  // copy cv information into ros message
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

void pubThread(DeviceInfo device)
{
  printf ("start thread: %d\n", device.device_num);
  VideoCapture cap(device.device_num);
  if (!cap.isOpened())
  {
    printf ("failed open device num: %d\n", device.device_num);
    return;
  }
  auto image_message = std::make_shared<sensor_msgs::msg::Image>();
  Mat image;
  int i = 0;
  while (rclcpp::ok())
  {
    cap>>image;
    convert_frame_to_message(image, i++, image_message);
    device.image_pub->publish(image_message);
  }
}

int main(int argc, char ** argv)
{

  int device_0_num = -1;
  int device_1_num = -1;
  int device_2_num = -1;
  int device_3_num = -1;
  int device_4_num = -1;

  printf("test1\n");
  rclcpp::init(argc, argv);
  auto cam_node = rclcpp::Node::make_shared("usb_cam");

  cam_node->get_parameter_or("device_0_num", device_0_num, NO_DEVICE_NUM);
  cam_node->get_parameter_or("device_1_num", device_1_num, NO_DEVICE_NUM);
  cam_node->get_parameter_or("device_2_num", device_2_num, NO_DEVICE_NUM);
  cam_node->get_parameter_or("device_3_num", device_3_num, NO_DEVICE_NUM);
  cam_node->get_parameter_or("device_4_num", device_4_num, NO_DEVICE_NUM);

  printf("test2\n");
  std::vector<DeviceInfo> run_pub;
  std::vector<boost::thread*> process_thread;
  if (device_0_num != -1)
  {
    DeviceInfo device;
    device.device_num = device_0_num;
    device.image_pub = cam_node->create_publisher<sensor_msgs::msg::Image>("/usb_cam_pub/image0");
    boost::thread device_thread = boost::thread(boost::bind(&pubThread, device));
    device_thread.detach();
  }

  if (device_1_num != -1)
  {
    DeviceInfo device;
    device.device_num = device_1_num;
    device.image_pub = cam_node->create_publisher<sensor_msgs::msg::Image>("/usb_cam_pub/image1");
    boost::thread device_thread = boost::thread(boost::bind(&pubThread, device));
    device_thread.detach();
  }

  if (device_2_num != -1)
  {
    DeviceInfo device;
    device.device_num = device_2_num;
    device.image_pub = cam_node->create_publisher<sensor_msgs::msg::Image>("/usb_cam_pub/image2");
    boost::thread device_thread = boost::thread(boost::bind(&pubThread, device));
    device_thread.detach();
  }

  if (device_3_num != -1)
  {
    DeviceInfo device;
    device.device_num = device_3_num;
    device.image_pub = cam_node->create_publisher<sensor_msgs::msg::Image>("/usb_cam_pub/image3");
    boost::thread device_thread = boost::thread(boost::bind(&pubThread, device));
    device_thread.detach();
  }

  if (device_4_num != -1)
  {
    DeviceInfo device;
    device.device_num = device_4_num;
    device.image_pub = cam_node->create_publisher<sensor_msgs::msg::Image>("/usb_cam_pub/image4");
    boost::thread device_thread = boost::thread(boost::bind(&pubThread, device));
    device_thread.detach();
  }
  printf("test3\n");
  rclcpp::WallRate loop_rate(50);

  while (rclcpp::ok())
  {
    loop_rate.sleep();
  }

  return 0;
}

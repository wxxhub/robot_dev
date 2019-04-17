#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <boost/thread.hpp>

using namespace std;

class Test
{
public:
  Test()
  {
    test_node_ = rclcpp::Node::make_shared("test2");
    queue_thread_ = boost::thread(std::bind(&Test::queueThread, this));
  }

  ~Test()
  {
    queue_thread_.join();
  }

private:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;

  boost::thread queue_thread_;

  void queueThread()
  {
    cout<<"Start queue..."<<endl;
    auto test_pub_ = test_node_->create_publisher<std_msgs::msg::String>("/test_topic");
    auto test_sub = test_node_->create_subscription<std_msgs::msg::String>("/test_topic", std::bind(&Test::testCallback, this, std::placeholders::_1));
    std_msgs::msg::String msg;
    rclcpp::WallRate loop_rata(10);
    int i = 0;
    while (rclcpp::ok())
    {
      msg.data = "Hello, This my message: " + std::to_string(++i);
      test_pub_->publish(msg);
      rclcpp::spin_some(test_node_);
      loop_rata.sleep();
    }
  }

  void testCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    cout<<"recive: "<<msg->data<<endl;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr main_node = rclcpp::Node::make_shared("main_node");
  Test *test = new Test();

  rclcpp::WallRate loop_rata(30);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(main_node);
    loop_rata.sleep();
  }

  return 0;
}

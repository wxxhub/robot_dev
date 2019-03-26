#常用API说明

## WallRate
> **rclcpp::WallRate loop_rate(rate)** rate为循环频率;

## ament_index_cpp::get_package_share_directory
> 获取莫个包的路径ROS中对应API为ros:: package::getPath
> CMakeLists.txt中添加 **find_package(ament_index_cpp REQUIRED)**
> package中添加 **<buildtool_depend>ament_index_cpp</buildtool_depend>**
> 头文件 **ament_index_cpp/get_package_share_directory.hpp**
> 示例
>> default_setting_path_ = ament_index_cpp::get_package_share_directory("ball_hsv_detector") + "/config/ball_detector_params.yaml";
> 注意！！！
>> 使用try,避免找不到路径报错。
>> 参考包 ball_hsv_detector
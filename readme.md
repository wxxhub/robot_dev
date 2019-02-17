####

#### 开发环境
> Ubantu18, ROS2-crystal
> ROS2不要使用bouncy版本，launch无法实现传参

#### yaml-cpp
> ```` sudo apt-get install libyaml-cpp-dev ````


#### regulator_module
> 监测电机情况,动态调整电机参数
> 发送异常信息
> 保存异常日志

#### Install
> 在install中安装文件夹，比如安装launch 文件夹
> # Install launch files.
> install(DIRECTORY
>   launch
>   DESTINATION share/${PROJECT_NAME}/
> )
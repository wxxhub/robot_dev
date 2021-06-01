#### 基于ROS2的机器人平台  

[开发日志](Explain/develop_log.md)  
[问题解决](Explain/problem_solve.md)  

#### 开发环境  
> [环境配置](Explain/enviroment.md)  
> Ubantu18, ROS2-crystal   
> ROS2-bouncy版本，launch无法实现传参      
> 而ROS2-crystal，没有ament,可以使用[colcon](Explain/colcon.md), 测试可以使用，需要改写部分配置  

#### 部署
```
mkdir robot_dev
cd robot_dev
git clone http://218.199.176.173:88/wxx/ros2_robot.git src
colcon build --symlink-install
```

#### Install
```
##在install中安装文件夹，比如安装launch 文件夹  
##**Install launch files.**
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
````

#### 目录
> [新增功能](Explain/new_function.md)  
> [视觉处理模块](Visual-Detector/readme.md)   
> [ROBOTIS-Framework](ROBOTIS-Framework/readme.md)  
> [ROBOTIS-Module](ROBOTIS-Module/readme.md)  
> [ROS2_API](Explain/api.md)

#### 快捷使用
```shell
ros2 topic pub /robotis/head_control/scan_command std_msgs/String "data: scan" # 终端发布消息
```

#### 设置串口权限  
```
sudo gedit /etc/udev/rules.d/70-ttyusb.rules  
## 添加： KERNEL=="ttyUSB[0-9]*",MODE="0666"  
## 保存，重新插入串口  
```

#### 弃用模块 
> online_walking_module 结果比较复杂, 几乎不使用   
> tuning_module 暂时没有用途  

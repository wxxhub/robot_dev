#### 基于ROS2的机器人平台  

[开发日志](Explain/develop_log.md)  
[问题解决](Explain/problem_solve.md)  

#### 开发环境  
> Ubantu18, ROS2-crystal   
> ROS2-bouncy版本，launch无法实现传参      
> 而ROS2-crystal，没有ament,可以使用[colcon](Explain/colcon.md), 测试可以使用，需要改写部分配置  

#### yaml-cpp  
> ```` sudo apt-get install libyaml-cpp-dev ````   

#### regulator_module  
> 监测电机情况,动态调整电机参数   
> 发送异常信息  
> 保存异常日志  

#### Install
> 在install中安装文件夹，比如安装launch 文件夹  
> **Install launch files.**
> ````
> install(DIRECTORY
>  launch
>  DESTINATION share/${PROJECT_NAME}/
> )
>````

#### 目录
> [视觉处理模块](Visual-Detector)  
> [ROBOTIS-Framework](ROBOTIS-Framework)  
> [ROBOTIS-OP3](ROBOTIS-OP3)  
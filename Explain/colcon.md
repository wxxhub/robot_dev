# colcon

#### 安装
> **sudo apt install python3-colcon-common-extensions**   

#### 使用
> **colcon build --symlink-install**  *编译所有文件*  
> **colcon build --packages-select dynamixel_sdk**  *编译指定文件，比如dynamixel_sdlk*   
> **colcon -h** *查看使用帮助*  

#### 注意事项
> 环境配置链接文件是 install 中的 **setup.bash**  (Linux/OS X)  
> call **install\setup.bat** (Windows)  
# colcon

#### 安装
> **sudo apt install python3-colcon-common-extensions**   

#### 使用
> **colcon build --symlink-install**  *编译所有文件*  
> **colcon build --symlink-install --packages-select behaviours**  *编译指定文件，比如behaviours*   
> **colcon -h** *查看使用帮助*  

#### 注意事项
> 环境配置链接文件是 install 中的 **setup.bash**  (Linux/OS X)  
> call **install\setup.bat** (Windows)  
> **--symlink-install**,是在install中生成链接文件，而不是复制过去。
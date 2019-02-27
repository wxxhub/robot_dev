# ROAD_DETECTOR

#### 关键参数
> 关键参数请在 **road_detector.cpp** 中的 **RoadDetector::RoadDetector()** 中修改  
> **road_color** 路线颜色，如需增加颜色请修改 **Color** 中的参数  
> **show_result_** 是否显示识别结果  
> **mark_detector** 是否识别路标  
> **wite_background_** 路标背景色__非黑即白  
> **mark_rect_width** 图标大致大小  

## 函数说明
#### 函数说明
> **road_detector.h** 
>> **imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)** 图片回调函数  
>> **process()** 图像处理主函数  
>> **detector()** 道路识别  
>> **getRoad()** 获取路线  
>> **getMarkImage(cv::Mat mark_image, float &road_angle)** 获取路标图片  
>> **showResult(cv::Mat image)** 显示识别结果  
>> **newImage()** 判断是否更新图片  
>> **publishResult()** 发布识别结果  
>> **encodingToMatType(const std::string & encoding)** 解码图片类型  
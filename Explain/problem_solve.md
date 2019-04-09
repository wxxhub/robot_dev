## 常见错误解决


#### [ERROR] [launch.LaunchService]: Caught exception in launch (see debug for traceback): 'utf-8' codec can't decode byte 0x91 in position 36: invalid start  
>  字符串编码问题，**std::String::msg**不能直接打印，需要加上 **.c_str()**

#### [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.  

## 常见错误解决


#### [ERROR] [launch.LaunchService]: Caught exception in launch (see debug for traceback): 'utf-8' codec can't decode byte 0x91 in position 36: invalid start  
>  字符串编码问题，**std::String::msg**不能直接打印，需要加上 **.c_str()**

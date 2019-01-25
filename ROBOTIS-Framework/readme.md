## ROBOTIS-Framework

#### 各话题(topic)的作用及类型
> |话题|消息类型|作用|
> |-|-|-|
> | /robotis/write_control_table |robotis_controller_msgs::WriteControlTable | 向设备（电机，开发板）同步写入数据（一个设备多个个item) |
> | /robotis/sync_write_item | robotis_controller_msgs::SyncWriteItem | 向设备（电机，开发板）同步写入数据（一个设备一个item) |
> | /robotis/set_joint_ctrl_modules | robotis_controller_msgs::JointCtrlModule | 设置电机名和当前模块名 |
> | /robotis/enable_ctrl_module | std_msgs::String | 打开模块 |
> | /robotis/set_control_mode | std_msgs::String | 设置控制模式，直接控制还是模块控制 |
> | /robotis/set_joint_states | sensor_msgs::JointState | 设置电机位置 |
> | /robotis/enable_offset | std_msgs::Bool | 控制is_offset_enabled_，是否打开补偿 |
> | ---- | ----- | ---- |
> | /robotis/goal_joint_states | sensor_msgs::JointState | 目标电机状态 |
> | /robotis/present_joint_states | sensor_msgs::JointState | 当前电机状态 |
> | /robotis/present_joint_ctrl_modules | robotis_controller_msgs::JointCtrlModule | 发布当前模块 |
> | ---- | ----- | ---- |
> | /robotis/get_present_joint_ctrl_modules | robotis_controller_msgs::GetJointModule | 获取电机名和当前模块名 |
> | /robotis/set_present_joint_ctrl_modules | robotis_controller_msgs::SetJointModule | 设置电机名和当前模块名 | 
> | /robotis/set_present_ctrl_modules | robotis_controller_msgs::SetModule | 设置模块和/robotis/enable_ctrl_modul类似 |
> | /robotis/present_joint_ctrl_modules | robotis_controller_msgs::LoadOffset | 加载补偿 |
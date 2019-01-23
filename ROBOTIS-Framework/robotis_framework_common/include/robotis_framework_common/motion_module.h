#ifndef ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_

#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"

namespace robotis_framework
{

enum ControlMode
{
    PositionControl,
    VelocityControl,
    TorqueControl
};

class MotionModule
{
protected:
    bool        enable_;
    std::string module_name_;
    ControlMode control_mode_;

public:
    std::map<std::string, DynamixelState *> result_;

    virtual ~MotionModule(){}

    std::string getModuleName()  { return module_name_; }
    ControlMode getControlMode() { return control_mode_; }

    void setModuleEnable(bool enable)
    {
        if (this->enable_ == enable)
            return;
        
        this->enable_ = enable;
        if (enable)
            onModuleEnable();
        else
            onModuleDisable();
    }

    bool getModuleEnable() { return enable_; }

    virtual void onModuleEnable() {}
    virtual void onModuleDisable() {}
};

}

#endif /* ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_ */
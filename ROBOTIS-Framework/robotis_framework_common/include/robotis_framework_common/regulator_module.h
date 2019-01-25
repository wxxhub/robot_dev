#ifndef ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_

#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"


namespace robotis_framework
{

class RegulatorModule
{
protected:
    bool enable_;
    std::string module_name_;

public:
    std::map<std::string, DynamixelState *> result_;

    virtual ~RegulatorModule(){}

    std::string getModuleName() { return module_name_; }
    bool getModuleEnable() { return enable_; }

    virtual void onModuleEnable(){}
    virtual void onModuleDisable(){}

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
};
    
} // robotis_framework


#endif /* ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_ */
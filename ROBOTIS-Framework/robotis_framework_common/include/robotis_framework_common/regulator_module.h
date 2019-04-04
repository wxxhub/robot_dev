#ifndef ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_

#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"
#include "dynamixel_sdk/group_bulk_read.h"


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

    virtual void  initialize(const int control_cycle_msec, Robot *robot) = 0;
    virtual void  process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors, std::map<std::string, dynamixel::GroupBulkRead *> port_to_bulk_read) = 0;

    virtual void	stop() = 0;
    virtual bool	isRunning() = 0;
};
    
} // robotis_framework


#endif /* ROBOTIS_FRAMEWORK_COMMON_REGULATOR_MODULE_H_ */
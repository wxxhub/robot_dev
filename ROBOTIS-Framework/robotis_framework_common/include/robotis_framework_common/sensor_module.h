#ifndef ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_


#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"

namespace robotis_framework
{

class SensorModule
{
protected:
  std::string module_name_;

public:
  std::map<std::string, double> result_;

  virtual ~SensorModule() { }

  std::string   getModuleName() { return module_name_; }

  virtual void  initialize(const int control_cycle_msec, Robot *robot) = 0;
  virtual void  process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors) = 0;
};

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_ */
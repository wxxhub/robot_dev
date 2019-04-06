#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

OpenCRModule::OpenCRModule()
    : DEBUG_PRINT(false)
{
}

OpenCRModule::~OpenCRModule()
{
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{

}

void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors)

{
    
}

}  // namespace robotis_op

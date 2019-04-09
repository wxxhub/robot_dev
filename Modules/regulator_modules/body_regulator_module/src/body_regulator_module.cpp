#include "body_regulator_module/body_regulator_module.h"

using namespace robotis_framework;

namespace robotis_op
{

BodyRegulatorModule::BodyRegulatorModule()
{
}

BodyRegulatorModule::~BodyRegulatorModule()
{
}

void BodyRegulatorModule::initialize(const int control_cycle_msec, Robot *robot)
{

}

void BodyRegulatorModule::process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors, std::map<std::string, dynamixel::GroupBulkRead *> port_to_bulk_read)
{
    static int speed_test = 0;
    for (auto& it : dxls)
    {
        Dynamixel *dxl = it.second;
        
        uint8_t error;
        port_to_bulk_read[dxl->port_name_]->getError(dxl->id_, &error);

        error_map_[dxl->id_] = error;
    }
    // printf("speed_test: %d\n", speed_test++);
    // printError();
}

void BodyRegulatorModule::stop()
{

}

bool BodyRegulatorModule::isRunning()
{

}

void BodyRegulatorModule::onModuleEnable()
{

}

void BodyRegulatorModule::onModuleDisable()
{
    
}

void BodyRegulatorModule::printError()
{
    printf("-------------BodyRegulatorError-------------\n");
    std::map<uint8_t, uint8_t>::iterator iter = error_map_.begin();

    for (; iter != error_map_.end(); iter++)
    {
        printf("%-3d|%-5d|\n", iter->first, int(iter->second));
    }

    printf("--------------------------------------------\n");
}

}  // namespace robotis_op

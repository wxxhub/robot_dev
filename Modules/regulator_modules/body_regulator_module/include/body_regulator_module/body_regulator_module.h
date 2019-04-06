#ifndef HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_
#define HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

#include "body_regulator_module/visibility_control.h"

#include "robotis_framework_common/regulator_module.h"

namespace robotis_op
{

class BodyRegulatorModule : public robotis_framework::RegulatorModule, public robotis_framework::Singleton<BodyRegulatorModule>
{
public:
  BodyRegulatorModule();

  virtual ~BodyRegulatorModule();
  void  initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void  process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors, std::map<std::string, dynamixel::GroupBulkRead *> port_to_bulk_read);

  void	stop();
  bool	isRunning();

  void onModuleEnable();
  void onModuleDisable();

  void printError();

private: 
  std::map<uint8_t, uint8_t> error_map_;
};

}  // namespace robotis_op

#endif  // HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

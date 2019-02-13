#ifndef HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_
#define HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

#include "hand_regulator_module/visibility_control.h"

#include "robotis_framework_common/regulator_module.h"

namespace robotis_op
{

class HandRegulatorModule : public robotis_framework::RegulatorModule, public robotis_framework::Singleton<HandRegulatorModule>
{
public:
  HandRegulatorModule();

  virtual ~HandRegulatorModule();
  void  initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void  process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void	stop();
  bool	isRunning();

  void onModuleEnable();
  void onModuleDisable();
};

}  // namespace robotis_op

#endif  // HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

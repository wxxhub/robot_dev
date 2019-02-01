#ifndef HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_
#define HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

#include "hand_regulator_module/visibility_control.h"

#include "robotis_framework_common/regulator_module.h"

namespace robotis_op
{

class HandRegulatorModule
{
public:
  HandRegulatorModule();

  virtual ~HandRegulatorModule();
};

}  // namespace robotis_op

#endif  // HAND_REGULATOR_MODULE__HAND_REGULATOR_MODULE_HPP_

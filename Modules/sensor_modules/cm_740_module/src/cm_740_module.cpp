#include "cm_740_module/cm_740_module.h"

using namespace robotis_op;

Cm740Module::Cm740Module()
    : control_cycle_msec_(8),
      DEBUG_PRINT(false),
      present_volt_(0.0)
{
  module_name_ = "cm_740_module";  // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;

  result_["present_voltage"] = 0.0;
  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;

//   last_msg_time_ = ros::Time::now();
}

Cm740Module::~Cm740Module()
{
}

cmake_minimum_required(VERSION 3.5)
project(op3_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(op3_online_walking_module_msgs REQUIRED)
find_package(op3_tuning_module_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(op3_balance_control REQUIRED)
find_package(open_cr_module REQUIRED)
find_package(op3_base_module REQUIRED)
find_package(op3_head_control_module REQUIRED)
find_package(op3_walking_module REQUIRED)
find_package(op3_action_module REQUIRED)
find_package(op3_direct_control_module REQUIRED)
find_package(op3_online_walking_module REQUIRED)
find_package(op3_tuning_module REQUIRED)
find_package(Boost REQUIRED)

add_executable(op3_manager src/op3_manager.cpp)
target_include_directories(op3_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  op3_manager
  "robotis_controller_msgs"
  "op3_action_module_msgs"
  "op3_walking_module_msgs"
  "op3_online_walking_module_msgs"
  "op3_tuning_module_msgs"
  "dynamixel_sdk"
  "robotis_framework_common"
  "robotis_device"
  "robotis_controller"
  "robotis_math"
  "op3_kinematics_dynamics"
  "op3_balance_control"
  "open_cr_module"
  "op3_base_module"
  "op3_head_control_module"
  "op3_walking_module"
  "op3_action_module"
  "op3_direct_control_module"
  "op3_online_walking_module"
  "op3_tuning_module"
)

install(TARGETS op3_manager
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

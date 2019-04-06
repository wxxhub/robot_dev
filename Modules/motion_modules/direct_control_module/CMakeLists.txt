cmake_minimum_required(VERSION 3.5)
project(direct_control_module)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(Boost REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_library(direct_control_module src/direct_control_module.cpp)
target_include_directories(direct_control_module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  direct_control_module
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "robotis_controller_msgs"
  "robotis_framework_common"
  "robotis_device"
  "robotis_math"
  "op3_kinematics_dynamics"
  "dynamixel_sdk"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(direct_control_module PRIVATE "OP3_DIRECT_CONTROL_MODULE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS direct_control_module
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_interfaces(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  direct_control_module
)

ament_package()

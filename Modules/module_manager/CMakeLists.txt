cmake_minimum_required(VERSION 3.5)
project(module_manager)

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
find_package(rclcpp REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(action_module_msgs REQUIRED)
find_package(walking_module_msgs REQUIRED)
find_package(online_walking_module_msgs REQUIRED)
find_package(tuning_module_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(op3_balance_control REQUIRED)
find_package(open_cr_module REQUIRED)
find_package(base_module REQUIRED)
find_package(head_control_module REQUIRED)
find_package(walking_module REQUIRED)
find_package(action_module REQUIRED)
find_package(direct_control_module REQUIRED)
find_package(online_walking_module REQUIRED)
find_package(tuning_module REQUIRED)
find_package(body_regulator_module REQUIRED)
find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
  message("boost not found")
endif()
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
#link_directories(/usr/local/lib)  

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_executable(module_manager src/module_manager.cpp)
target_link_libraries(${PROJECT_NAME}
boost_system
boost_thread
${Boost_LIBRARIES}
yaml-cpp
${YAML_CPP_LIBRARIES}
)
target_include_directories(module_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
  /usr/local/include/yaml-cpp)
ament_target_dependencies(
  module_manager
  "rclcpp"
  "robotis_controller_msgs"
  "action_module_msgs"
  "walking_module_msgs"
  "online_walking_module_msgs"
  "tuning_module_msgs"
  "robotis_framework_common"
  "robotis_device"
  "robotis_controller"
  "robotis_math"
  "op3_kinematics_dynamics"
  "op3_balance_control"
  "open_cr_module"
  "base_module"
  "head_control_module"
  "walking_module"
  "action_module"
  "direct_control_module"
  "online_walking_module"
  "tuning_module"
  "body_regulator_module"
)

IF (CMAKE_SYSTEM_NAME MATCHES "Windows") 
  message("Now is windows") 
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Apple") 
  message("Now is Apple systens.") 
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(module_manager dxl_x64_cpp)
ENDIF ()

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)


install(TARGETS module_manager
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()

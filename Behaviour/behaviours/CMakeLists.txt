cmake_minimum_required(VERSION 3.5)
project(behaviours)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(walking_module_msgs REQUIRED)
find_package(action_module_msgs REQUIRED)
find_package(robotis_math REQUIRED)

add_library(behaviours src/behaviour_demo/behaviour_demo.cpp)
target_link_libraries(${PROJECT_NAME}
  boost_system
  boost_thread
  ${Boost_LIBRARIES}
  yaml-cpp
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(behaviours PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  behaviours
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "robotis_controller_msgs"
  "walking_module_msgs"
  "action_module_msgs"
  "robotis_math"
  "ament_index_cpp"
)

target_compile_definitions(behaviours PRIVATE "BEHACIOURS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS behaviours
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(behaviour_manager src/behaviour_manager.cpp)
target_include_directories(behaviour_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(behaviour_manager behaviours)

install(TARGETS behaviour_manager
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_interfaces(
  export_${PROJECT_NAME}
)

ament_export_libraries(
  behaviours
)

ament_package()

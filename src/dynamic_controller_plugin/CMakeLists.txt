cmake_minimum_required(VERSION 3.8)
project(dynamic_controller_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)

# pluginlib_export_plugin_description_file(dynamic_controller_base plugins.xml)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

#add_library(dynamic_controller_plugin src/dynamic_controller_plugin.cpp src/computed_torque_controller.cpp)
add_library(dynamic_controller_plugin src/computed_torque_controller.cpp)
add_library(dynamic_controller_plugin::dynamic_controller_plugin ALIAS dynamic_controller_plugin)
target_compile_features(dynamic_controller_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(dynamic_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(dynamic_controller_plugin
  pluginlib
  rclcpp
  std_msgs
  orocos_kdl_vendor
  orocos_kdl
  kdl_parser
  controller_interface
  hardware_interface
  Eigen3
  trajectory_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dynamic_controller_plugin PRIVATE "DYNAMIC_CONTROLLER_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS dynamic_controller_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  dynamic_controller_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

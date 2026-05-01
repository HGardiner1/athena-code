cmake_minimum_required(VERSION 3.8)
project(ccd_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  umdloop_can_library
  realtime_tools
  msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_export_dependencies(rosidl_default_runtime)

ament_auto_find_build_dependencies()

include_directories(include)

add_library(
  ccd_ros2_control
  SHARED
  src/ccd_hardware_interface.cpp
)

target_compile_features(ccd_ros2_control PUBLIC cxx_std_20)
target_include_directories(ccd_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ccd_ros2_control>
)

# Link CAN library
target_link_libraries(ccd_ros2_control PUBLIC umdloop_can_library::umdloop_can_library)

ament_target_dependencies(
  ccd_ros2_control PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  msgs
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ccd_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/ccd_ros2_control
)

install(TARGETS ccd_ros2_control
  EXPORT export_ccd_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_ccd_ros2_control)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

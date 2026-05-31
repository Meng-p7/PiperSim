cmake_minimum_required(VERSION 3.8)
project(piper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ── C++ hardware interface plugin ─────────────────────────────────────────────
add_library(piper_hardware SHARED src/piper_hardware.cpp)
target_include_directories(piper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(piper_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface piper_control_hardware.xml)

install(
  TARGETS piper_hardware
  EXPORT piper_hardwareTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# ── Python package ────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}
)

# ── Data files ────────────────────────────────────────────────────────────────
install(FILES piper_control_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# ── ament exports ─────────────────────────────────────────────────────────────
ament_export_targets(piper_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()

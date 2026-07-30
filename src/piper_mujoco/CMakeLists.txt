cmake_minimum_required(VERSION 3.8)
project(piper_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install models (symlink to official MuJoCo model)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  scripts/digital_twin_sync_realtime.py
  scripts/run_digital_twin.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

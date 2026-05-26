cmake_minimum_required(VERSION 3.8)
project(piper_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

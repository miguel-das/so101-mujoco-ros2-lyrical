cmake_minimum_required(VERSION 3.20)
project(so101_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    mujoco
    urdf
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
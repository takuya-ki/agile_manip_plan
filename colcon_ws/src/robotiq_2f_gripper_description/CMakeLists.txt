cmake_minimum_required(VERSION 3.22)
project(robotiq_2f_gripper_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.10)
project(isaac_ros_common)
find_package(ament_cmake REQUIRED)

# Scripts/ and cmake/ are copied from the upstream isaac_ros_common
# package by ``docker/Dockerfile`` at image-build time, so cuMotion can
# resolve ``isaac_ros_common_scripts_path`` / ``isaac_ros_common_cmake_path``
# without pulling in the real isaac_ros_common (which depends on VPI).
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)
install(DIRECTORY cmake/    DESTINATION share/${PROJECT_NAME}/cmake)

set(scripts_path "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/scripts")
set(cmake_path "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake")
ament_index_register_resource(isaac_ros_common_scripts_path CONTENT "${scripts_path}")
ament_index_register_resource(isaac_ros_common_cmake_path    CONTENT "${cmake_path}")

ament_package(CONFIG_EXTRAS
  cmake/isaac_ros_common-version-info.cmake
  cmake/isaac_ros_common-extras.cmake
  cmake/isaac_ros_common-extras-modules.cmake
  cmake/isaac_ros_common-extras-assets.cmake
)

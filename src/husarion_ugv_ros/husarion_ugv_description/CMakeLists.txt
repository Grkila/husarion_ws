cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
        DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

ament_package()

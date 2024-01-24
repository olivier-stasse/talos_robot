cmake_minimum_required(VERSION 3.8)
project(talos_description)

find_package(ament_cmake_auto REQUIRED xacro)

ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE)

ament_auto_package(INSTALL_TO_SHARE config gazebo launch meshes robots ros2_control srdf urdf test)


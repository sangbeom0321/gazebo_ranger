cmake_minimum_required(VERSION 3.8)
project(gazebo_ranger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(rosgraph_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_scan src/laser_scan.cpp)
ament_target_dependencies(laser_scan 
  rclcpp 
  std_msgs 
  sensor_msgs 
  pcl_conversions 
  nav_msgs
  rosgraph_msgs
)
target_link_libraries(laser_scan
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
install(TARGETS
  laser_scan
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config launch meshes models urdf worlds rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules (scripts are installed via install(PROGRAMS) below)
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/ranger_controller_key.py
    scripts/keyboard_teleop.py
    scripts/wheel_odometry.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

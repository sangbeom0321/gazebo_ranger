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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Find nlohmann_json for control node
find_package(nlohmann_json REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include directories for control headers
include_directories(include)

add_executable(state_machine src/state_machine.cpp)
ament_target_dependencies(state_machine rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs gazebo_msgs)
install(TARGETS
  state_machine
  DESTINATION lib/${PROJECT_NAME})

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

# Add car_control executable
add_executable(car_control src/car_control.cpp)
ament_target_dependencies(car_control 
  rclcpp 
  std_msgs 
  nav_msgs 
  geometry_msgs 
  sensor_msgs
)
target_include_directories(car_control PRIVATE include)
target_link_libraries(car_control ${rclcpp_LIBRARIES} Eigen3::Eigen)
install(TARGETS
  car_control
  DESTINATION lib/${PROJECT_NAME})

# Install control headers and JSON config
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)
install(FILES include/control/A.json
  DESTINATION share/${PROJECT_NAME}/include/control
)

install(DIRECTORY map config launch meshes models urdf worlds rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules (scripts are installed via install(PROGRAMS) below)
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/set_entity_state.py
    scripts/ranger_controller.py
    scripts/ranger_controller_key.py
    scripts/keyboard_teleop.py
    scripts/utm_publisher.py
    scripts/pub_states.py
    scripts/path_debug/path_publisher.py
    scripts/base_link_state.py
    scripts/mappub.py
    scripts/path_debug/local_path_publisher.py
    scripts/box_keyboard_controller.py
    scripts/marker_based_localizer.py
    scripts/fake_local_path.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rby1_teleop_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

# Suppress Pinocchio/HPP-FCL warnings
add_definitions(-DCOAL_DISABLE_HPP_FCL_WARNINGS)

# Pinocchio path
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Executable
add_executable(teleop_ik_node teleop_ik.cpp)

# Use ament_target_dependencies for ROS packages
ament_target_dependencies(teleop_ik_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
)

# Link non-ament dependencies
target_include_directories(teleop_ik_node PRIVATE 
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)
target_link_libraries(teleop_ik_node
  ${pinocchio_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

install(TARGETS
  teleop_ik_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

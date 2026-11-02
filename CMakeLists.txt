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
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

set(RBY1_PINOCCHIO_LIBRARIES ${pinocchio_LIBRARIES})
if(TARGET pinocchio::pinocchio)
  set(RBY1_PINOCCHIO_LIBRARIES pinocchio::pinocchio)
endif()

# Executable
add_executable(teleop_ik_node src/ik_solver/teleop_ik.cpp)
add_executable(proprio_pose_node src/ik_solver/proprio_pose_node.cpp)

# Use ament_target_dependencies for ROS packages
ament_target_dependencies(teleop_ik_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  geometry_msgs
  trajectory_msgs
)
ament_target_dependencies(proprio_pose_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Link non-ament dependencies
target_include_directories(teleop_ik_node PRIVATE 
  include
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)
target_link_libraries(teleop_ik_node
  ${RBY1_PINOCCHIO_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)
target_include_directories(proprio_pose_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)
target_link_libraries(proprio_pose_node
  ${RBY1_PINOCCHIO_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

install(TARGETS
  teleop_ik_node
  proprio_pose_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/for_test/sim_pose_publisher.py
  src/for_test/ee_parquet_pose_player.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY model samples
  DESTINATION share/${PROJECT_NAME}
)
install(FILES
  display.launch.py
  rby1.rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_pose_input_conversion test/test_pose_input_conversion.cpp)
  target_include_directories(test_pose_input_conversion PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_pose_input_conversion
    geometry_msgs
    std_msgs
  )
  target_link_libraries(test_pose_input_conversion
    ${RBY1_PINOCCHIO_LIBRARIES}
  )

  ament_add_gtest(test_joint_state_utils test/test_joint_state_utils.cpp)
  target_include_directories(test_joint_state_utils PRIVATE
    include
  )
  ament_target_dependencies(test_joint_state_utils
    sensor_msgs
  )

  add_test(
    NAME test_ee_parquet_pose_player
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ee_parquet_pose_player.py
  )

  add_test(
    NAME test_active_topic_contract
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_active_topic_contract.py
  )

  add_test(
    NAME test_sample_episode
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/examples/mujoco_replay/replay_episode.py --validate-only
  )
endif()

ament_package()

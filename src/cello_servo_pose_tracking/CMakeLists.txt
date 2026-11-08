cmake_minimum_required(VERSION 3.8)
project(cello_servo_pose_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# moveit_servo::pose_tracking was removed in Jazzy: build this node only where it exists.
set(NODES wrist_tf_to_target_pose_node joint_trajectory_action_bridge_node)
if(TARGET moveit_servo::pose_tracking)
  add_executable(xr_pose_tracking_node src/xr_pose_tracking_node.cpp)
  target_compile_features(xr_pose_tracking_node PUBLIC cxx_std_17)
  ament_target_dependencies(
    xr_pose_tracking_node
    moveit_ros_planning
    moveit_servo
    control_msgs
    rclcpp
    rclcpp_action
  )
  target_link_libraries(
    xr_pose_tracking_node
    moveit_servo::pose_tracking
    moveit_servo::moveit_servo_lib_parameters
  )
  list(APPEND NODES xr_pose_tracking_node)
else()
  message(WARNING "moveit_servo::pose_tracking not found: skipping xr_pose_tracking_node")
endif()

add_executable(wrist_tf_to_target_pose_node src/wrist_tf_to_target_pose_node.cpp)
target_compile_features(wrist_tf_to_target_pose_node PUBLIC cxx_std_17)
ament_target_dependencies(
  wrist_tf_to_target_pose_node
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(joint_trajectory_action_bridge_node src/joint_trajectory_action_bridge_node.cpp)
target_compile_features(joint_trajectory_action_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(
  joint_trajectory_action_bridge_node
  control_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
)

install(TARGETS ${NODES}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

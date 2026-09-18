cmake_minimum_required(VERSION 3.8)
project(robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(named_goal src/tests/named_goal.cpp)
ament_target_dependencies(named_goal rclcpp moveit_ros_planning_interface)

add_executable(joint_goal src/tests/joint_goal.cpp)
ament_target_dependencies(joint_goal rclcpp moveit_ros_planning_interface)

add_executable(position_goal src/tests/position_goal.cpp)
ament_target_dependencies(position_goal rclcpp moveit_ros_planning_interface)

add_executable(cartesian_path src/tests/cartesian_path.cpp)
ament_target_dependencies(cartesian_path rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces robot_interfaces geometry_msgs tf2)

install(TARGETS
  named_goal
  joint_goal
  position_goal
  cartesian_path
  commander
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/tomato_picker.py
  src/sequencer.py
  src/tests/test_multi_tomato.py
  src/tests/test_button_server.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

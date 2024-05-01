cmake_minimum_required(VERSION 3.8)
project(robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_grasping REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(/usr/include/python3.10)

include_directories(/usr/include/eigen3)

add_executable(localPlanner src/LocalPlanner.cpp)
ament_target_dependencies(localPlanner rclcpp std_msgs nav_msgs tf2 rclcpp_action geometry_msgs)

add_executable(armController src/ArmController.cpp)
ament_target_dependencies(armController rclcpp std_msgs sensor_msgs controller_manager rclcpp_action)

add_executable(dynGrasp src/Integrated.cpp)
ament_target_dependencies(dynGrasp rclcpp std_msgs nav_msgs geometry_msgs rclcpp_action ros2_grasping)

set(srv_files "srv/MapPath.srv")
set(action_files "action/ArmControl.action" "action/BaseControl.action")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES nav_msgs std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dynGrasp "${cpp_typesupport_target}")
target_link_libraries(localPlanner "${cpp_typesupport_target}")
target_link_libraries(armController "${cpp_typesupport_target}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch map 
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
dynGrasp localPlanner armController 
  DESTINATION lib/${PROJECT_NAME})
ament_package()

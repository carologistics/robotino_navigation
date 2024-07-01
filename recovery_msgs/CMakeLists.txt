cmake_minimum_required(VERSION 3.8)
project(recovery_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
)

set(action_files
  "action/RecoverPose.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()

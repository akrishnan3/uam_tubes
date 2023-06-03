cmake_minimum_required(VERSION 3.8)
project(uam_operator_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

# generate interface
rosidl_generate_interfaces(${PROJECT_NAME}  "msg/FlightPlan.msg" DEPENDENCIES nav_msgs)

# 
ament_export_dependencies(rosidl_default_runtime nav_msgs)
ament_package()

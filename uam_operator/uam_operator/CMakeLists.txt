cmake_minimum_required(VERSION 3.8)
project(uam_operator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uam_flightmanager_msgs REQUIRED)

include_directories(include)

set(node_plugins "")

add_library(trajectory_server SHARED src/trajectory_server.cpp)
target_compile_definitions(trajectory_server PRIVATE "COMPOSITION_BUILDING_DLL")
#target_include_directories(trajectory_server PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_compile_features(trajectory_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  trajectory_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "visualization_msgs"
  "uam_flightmanager_msgs"
)

rclcpp_components_register_nodes(
  trajectory_server 
  "uam_operator::TubeTrajectoryServer"
  )

set(node_plugins "${node_plugins}uam_operator::TubeTrajectoryServer;$<TARGET_FILE:trajectory_server>\n")

add_library(vehicle_interface SHARED src/vehicle_interface.cpp)
target_compile_definitions(vehicle_interface PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(
  vehicle_interface
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "visualization_msgs"
  "uam_flightmanager_msgs"
)

rclcpp_components_register_nodes(vehicle_interface "uam_operator::VehicleInterface")
set(node_plugins "${node_plugins}uam_operator::VehicleInterface;$<TARGET_FILE:vehicle_interface>\n")

install(TARGETS
  trajectory_server
  vehicle_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
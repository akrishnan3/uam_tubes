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
find_package(nav_msgs REQUIRED)
find_package(uam_flightmanager_msgs REQUIRED)
find_package(uam_operator_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include)

set(node_plugins "")

add_library(preplanned_server SHARED src/preplanned_server.cpp)
target_compile_definitions(preplanned_server PRIVATE "COMPOSITION_BUILDING_DLL")

target_compile_features(preplanned_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  preplanned_server
  "rclcpp"
  "rclcpp_components"
  "visualization_msgs"
  "uam_flightmanager_msgs"
)

rclcpp_components_register_nodes(
  preplanned_server 
  "uam_operator::TrajectoryServer"
  )

set(node_plugins "${node_plugins}uam_operator::TrajectoryServer;$<TARGET_FILE:preplanned_server>\n")


add_library(tube_server SHARED src/tube_server.cpp src/tube_trajectory.cpp)
target_compile_definitions(tube_server PRIVATE "COMPOSITION_BUILDING_DLL")

target_compile_features(tube_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  tube_server
  "rclcpp"
  "rclcpp_components"
  "Eigen3"
  "geometric_shapes"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "uam_operator_msgs"
)

rclcpp_components_register_nodes(
  tube_server 
  "uam_operator::TubeServer"
  )

set(node_plugins "${node_plugins}uam_operator::TubeServer;$<TARGET_FILE:tube_server>\n")



add_library(operator_interface SHARED src/operator_interface.cpp)
target_compile_definitions(operator_interface PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(
  operator_interface
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "visualization_msgs"
  "nav_msgs"
  "uam_flightmanager_msgs"
  "uam_operator_msgs"
)

rclcpp_components_register_nodes(operator_interface "uam_operator::OperatorInterface")
set(node_plugins "${node_plugins}uam_operator::OperatorInterface;$<TARGET_FILE:operator_interface>\n")

#add_library(tube_trajectory SHARED src/tube_trajectory.cpp)
#target_compile_features(tube_trajectory PUBLIC c_std_99 cxx_std_17)
#ament_target_dependencies(
#  tube_trajectory
#  "rclcpp"
#  "Eigen3"
#  "geometric_shapes"
#  "geometry_msgs"
#  "visualization_msgs"
#)


#add_executable(random_paths src/random_paths.cpp)
#target_include_directories(random_paths PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

#target_compile_features(random_paths PUBLIC c_std_99 cxx_std_17)

#ament_target_dependencies(
#  random_paths
#  "rclcpp"
#  "Eigen3"
#  "geometric_shapes"
#  "visualization_msgs"
#)


install(TARGETS
  preplanned_server
  tube_server
  operator_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install(TARGETS random_paths
#  DESTINATION lib/${PROJECT_NAME})


#install(TARGETS
#  tube_trajectory
#  EXPORT export_tube_trajectory
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

#install(TARGETS tube_test
#  DESTINATION lib/${PROJECT_NAME})

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
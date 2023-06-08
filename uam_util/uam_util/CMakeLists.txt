cmake_minimum_required(VERSION 3.8)
project(uam_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(vicon_receiver REQUIRED)

add_executable(vehicle_odometry_bridge src/vehicle_odometry_bridge.cpp)
add_executable(vicon_odometry_bridge src/vicon_odometry_bridge.cpp)

target_include_directories(vehicle_odometry_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle_odometry_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  vehicle_odometry_bridge
  "rclcpp"
  "px4_msgs"
  "tf2_msgs"
  "px4_ros_com"
)

ament_target_dependencies(
  vicon_odometry_bridge
  "rclcpp"
  "px4_msgs"
  "vicon_receiver"
  "px4_ros_com"
)

install(TARGETS vehicle_odometry_bridge
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS vicon_odometry_bridge
  DESTINATION lib/${PROJECT_NAME})

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

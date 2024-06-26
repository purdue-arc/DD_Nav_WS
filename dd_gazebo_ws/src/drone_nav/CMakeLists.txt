cmake_minimum_required(VERSION 3.8)
project(drone_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

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

# Include Cpp "include" directory
include_directories(include/drone_nav)

add_executable(stabilized_tf_broadcaster src/stabilized_tf_broadcaster.cpp)
ament_target_dependencies(stabilized_tf_broadcaster geometry_msgs sensor_msgs rclcpp tf2 tf2_ros)
add_executable(footprint_tf_broadcaster src/footprint_tf_broadcaster.cpp)
ament_target_dependencies(footprint_tf_broadcaster geometry_msgs px4_msgs rclcpp tf2 tf2_ros)
add_executable(odom_publisher_broadcaster src/odom_publisher_broadcaster.cpp)
ament_target_dependencies(odom_publisher_broadcaster geometry_msgs px4_msgs px4_ros_com nav_msgs rclcpp tf2 tf2_ros)


# Install Cpp executables
install(TARGETS
  stabilized_tf_broadcaster
  footprint_tf_broadcaster
  odom_publisher_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src launch config behavior_tree description
  DESTINATION share/${PROJECT_NAME})

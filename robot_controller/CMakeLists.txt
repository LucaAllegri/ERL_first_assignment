cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(robot_interfaces REQUIRED)
  find_package(aruco_opencv_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  

  add_executable(robot_motion src/robot_motion.cpp)
  add_executable(perception_node src/perception_node.cpp)

  ament_target_dependencies(robot_motion rclcpp geometry_msgs)
  ament_target_dependencies(perception_node rclcpp geometry_msgs robot_interfaces aruco_opencv_msgs sensor_msgs cv_bridge)

  install(TARGETS
      robot_motion
      perception_node
      DESTINATION lib/${PROJECT_NAME}
  )


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

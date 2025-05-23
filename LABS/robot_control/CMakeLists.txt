cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(bug_control_node src/bug_control.cpp)
ament_target_dependencies(bug_control_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
)

add_executable(wall_control_node src/wall_control.cpp)
ament_target_dependencies(wall_control_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  bug_control_node
  wall_control_node
  DESTINATION lib/${PROJECT_NAME}
)
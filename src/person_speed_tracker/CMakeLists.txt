cmake_minimum_required(VERSION 3.8)
project(person_speed_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonInfo.msg"
  "msg/PersonInfoArray.msg"
  DEPENDENCIES std_msgs geometry_msgs gb_visual_detection_3d_msgs builtin_interfaces
)

# Add executable
add_executable(person_speed_tracker_node src/person_speed_tracker_node.cpp)
ament_target_dependencies(person_speed_tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  gb_visual_detection_3d_msgs
)

# Link generated messages (using newer method)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(person_speed_tracker_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  person_speed_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config (if any)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
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

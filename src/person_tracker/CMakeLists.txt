cmake_minimum_required(VERSION 3.8)
project(person_tracker)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonInfo.msg"
  "msg/PersonInfoArray.msg"
  "msg/HumanCluster.msg"
  "msg/HumanClusterArray.msg"
  DEPENDENCIES std_msgs geometry_msgs gb_visual_detection_3d_msgs builtin_interfaces
)

# Add clustering library
add_library(human_clusterer SHARED src/human_clusterer.cpp)
target_include_directories(human_clusterer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(human_clusterer PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
)

# Add executable
add_executable(person_tracker_node src/person_tracker_node.cpp)
target_include_directories(person_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(person_tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  gb_visual_detection_3d_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)
target_link_libraries(person_tracker_node human_clusterer)
set_target_properties(person_tracker_node PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH FALSE
)

# Link generated messages (using newer method)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(person_tracker_node "${cpp_typesupport_target}")

# Add visualizer executable
add_executable(person_tracker_visualizer src/person_tracker_visualizer.cpp)
target_include_directories(person_tracker_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(person_tracker_visualizer
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(person_tracker_visualizer "${cpp_typesupport_target}")
set_target_properties(person_tracker_visualizer PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH FALSE
)

# Install executable and library to the same directory
install(TARGETS
  person_tracker_node
  person_tracker_visualizer
  human_clusterer
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files, config, and rviz (if any)
install(DIRECTORY
  launch
  config
  rviz
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

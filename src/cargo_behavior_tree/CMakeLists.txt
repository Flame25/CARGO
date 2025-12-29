cmake_minimum_required(VERSION 3.8)
project(cargo_behavior_tree)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  behaviortree_cpp
  rclcpp
  rclcpp_action
  cargo_core
  cargo_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# Set source files
set(PLUGINS_CPP_FILES
  plugins/action/takeoff_action.cpp
  plugins/action/arm_service.cpp
  plugins/action/go_to_action.cpp
  plugins/action/go_to_gps_action.cpp
  plugins/action/offboard_service.cpp
  plugins/action/land_action.cpp
  plugins/condition/is_flying_condition.cpp
  plugins/decorator/wait_for_event.cpp
  plugins/decorator/wait_for_alert.cpp
  plugins/decorator/wait_for_time.cpp

)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/cargo_behavior_tree_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Create the library
add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the resource directory
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
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

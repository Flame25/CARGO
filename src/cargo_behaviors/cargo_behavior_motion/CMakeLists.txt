cmake_minimum_required(VERSION 3.8)
project(cargo_behavior_motion)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_action
  cargo_core
  cargo_msgs
  std_srvs
  cargo_behavior
  cargo_motion_reference_handlers
  geometry_msgs
  Eigen3
  rclcpp_components
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Plugins
set(PLUGINS_CPP_FILES
  land_behavior/plugins/land_plugin_platform.cpp
  takeoff_behavior/plugins/takeoff_plugin_platform.cpp
  goto_behavior/plugins/goto_plugin_position.cpp
)

# Library
add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Executable
set(SUBPACKAGES
  land_behavior
  takeoff_behavior
  goto_behavior
)

foreach(EXECUTABLE ${SUBPACKAGES})
  include_directories(
    ${EXECUTABLE}/include
    ${EXECUTABLE}/include/${PROJECT_NAME}
  )

  add_executable(${EXECUTABLE}_node ${EXECUTABLE}/src/${EXECUTABLE}.cpp ${EXECUTABLE}/src/${EXECUTABLE}_node.cpp)

  target_include_directories(${EXECUTABLE}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${EXECUTABLE}_node ${PROJECT_DEPENDENCIES})

  install(TARGETS ${EXECUTABLE}_node
    DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY ${EXECUTABLE}/include/
    DESTINATION include
  )

  install(
    DIRECTORY ${EXECUTABLE}/config/
    DESTINATION share/${PROJECT_NAME}/${EXECUTABLE}/config/
  )

  ament_export_include_directories(
    include
  )
endforeach()

# Components
add_library(takeoff_component SHARED
  takeoff_behavior/src/takeoff_behavior.cpp
)
ament_target_dependencies(takeoff_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(takeoff_component "TakeoffBehavior")

add_library(land_component SHARED
  land_behavior/src/land_behavior.cpp
)
ament_target_dependencies(land_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(land_component "LandBehavior")


add_library(goto_component SHARED
  goto_behavior/src/goto_behavior.cpp
)
ament_target_dependencies(goto_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(goto_component "GoToBehavior")

install(TARGETS
  takeoff_component
  land_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

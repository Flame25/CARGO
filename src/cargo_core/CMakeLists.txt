cmake_minimum_required(VERSION 3.8)
project(cargo_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# find dependencies
set(PROJECT_DEPENDENCIES
    ament_cmake
    rclcpp
    std_msgs
    nav_msgs
    sensor_msgs
    std_srvs
    geometry_msgs
    cargo_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    Eigen3
    rclcpp_lifecycle
    rclcpp_action
    image_transport
    cv_bridge
    yaml-cpp
    GeographicLib
    geographic_msgs)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(SOURCE_CPP_FILES
    src/rate.cpp
    src/node.cpp
    src/sensor.cpp
    src/core_functions.cpp
    src/platform_state_machine.cpp
    src/aerial_platform.cpp
    src/utils/tf_utils.cpp
    src/utils/frame_utils.cpp
    src/utils/control_mode_utils.cpp
    src/utils/yaml_utils.cpp
    src/utils/gps_utils.cpp) 

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  GeographicLib
)

# Mocks library
set(MOCKS_LIBRARY_NAME mocks)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIRS}
)


# Export dependencies so the executable can see them
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

add_executable(node_test src/main.cpp)

# Link it to your library (cargo_core) and necessary ROS 2 deps
target_link_libraries(node_test ${PROJECT_NAME})
# We specifically add rclcpp again for the executable to ensure clean linking
ament_target_dependencies(node_test rclcpp)

# --- CHANGE 3: Install the executable ---
# This allows 'ros2 run cargo_core node_test' to work
install(
  TARGETS node_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES} yaml-cpp GeographicLib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


# Python bindings
pybind11_add_module(cargo_names src/_cargo_core_pybind11.cpp)
target_include_directories(cargo_names PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(cargo_names PRIVATE
  rclcpp::rclcpp
)

install(
  TARGETS cargo_names
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()



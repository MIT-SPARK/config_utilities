cmake_minimum_required(VERSION 3.22.1)
project(config_utilities_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(config_utilities REQUIRED)
find_package(config_utilities_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Library
add_library(${PROJECT_NAME}
  src/ros_dynamic_config_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
target_link_libraries(${PROJECT_NAME} PUBLIC config_utilities::config_utilities)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  std_msgs
  config_utilities_msgs
)

# Executables
add_executable(demo_ros_dynamic_config_server demos/demo_ros_dynamic_config_server.cpp)
target_link_libraries(demo_ros_dynamic_config_server ${PROJECT_NAME})

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS demo_ros_dynamic_config_server
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS app/gui_node.py
        RENAME gui
        DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  config_utilities
  config_utilities_msgs
)
ament_package()

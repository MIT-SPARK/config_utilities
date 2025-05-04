if(ENABLE_Eigen3)
  add_executable(demo_config demo_config.cpp)
  target_link_libraries(demo_config ${PROJECT_NAME})
endif()

add_executable(demo_factory demo_factory.cpp)
target_link_libraries(demo_factory ${PROJECT_NAME})

add_executable(demo_inheritance demo_inheritance.cpp)
target_link_libraries(demo_inheritance ${PROJECT_NAME})

if (ENABLE_Eigen3 AND ENABLE_rclcpp)
  find_package(std_msgs REQUIRED)
  add_executable(demo_ros_dynamic_config_server demo_ros_dynamic_config_server.cpp)
  ament_target_dependencies(demo_ros_dynamic_config_server rclcpp std_msgs )
  target_link_libraries(demo_ros_dynamic_config_server ${PROJECT_NAME})

  install(TARGETS
  demo_ros_dynamic_config_server
  DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
endif()


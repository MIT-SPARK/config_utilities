if(ENABLE_Eigen3)
  add_executable(demo_config demo_config.cpp)
  target_link_libraries(demo_config ${PROJECT_NAME})
endif()

add_executable(demo_factory demo_factory.cpp)
target_link_libraries(demo_factory ${PROJECT_NAME})

add_executable(demo_inheritance demo_inheritance.cpp)
target_link_libraries(demo_inheritance ${PROJECT_NAME})

if (ENABLE_Eigen3 AND ENABLE_rclcpp)
  add_executable(demo_ros_dynamic_config_server demo_ros_dynamic_config_server.cpp)
  target_link_libraries(demo_ros_dynamic_config_server ${PROJECT_NAME})

  include(GNUInstallDirs)
  install(TARGETS demo_ros_dynamic_config_server
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
endif()


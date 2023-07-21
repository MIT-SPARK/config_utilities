if(CONFIG_UTILS_BUILD_DEMOS)

  if(ENABLE_Eigen3)
    add_executable(demo_config config.cpp)
    target_link_libraries(demo_config ${PROJECT_NAME})
  endif()

  add_executable(demo_factory factory.cpp)
  target_link_libraries(demo_factory ${PROJECT_NAME})

  add_executable(demo_inheritance inheritance.cpp)
  target_link_libraries(demo_inheritance ${PROJECT_NAME})

  if(ENABLE_Eigen3 AND ENABLE_roscpp)
    add_executable(demo_ros ros.cpp)
    target_link_libraries(demo_ros ${PROJECT_NAME})
  endif()

endif()

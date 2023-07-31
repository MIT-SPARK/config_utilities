if(ENABLE_Eigen3)
  add_executable(demo_config demo_config.cpp)
  target_link_libraries(demo_config ${PROJECT_NAME})
endif()

add_executable(demo_factory demo_factory.cpp)
target_link_libraries(demo_factory ${PROJECT_NAME})

add_executable(demo_inheritance demo_inheritance.cpp)
target_link_libraries(demo_inheritance ${PROJECT_NAME})

if(ENABLE_Eigen3 AND ENABLE_roscpp)
  add_executable(demo_ros demo_ros.cpp)
  target_link_libraries(demo_ros ${PROJECT_NAME})

  include(GNUInstallDirs)
  install(TARGETS demo_ros
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
endif()

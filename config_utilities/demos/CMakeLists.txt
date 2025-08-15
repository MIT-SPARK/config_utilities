if(Eigen3_FOUND)
  add_executable(demo_config demo_config.cpp)
  target_link_libraries(demo_config ${PROJECT_NAME})
endif()

add_executable(demo_factory demo_factory.cpp)
target_link_libraries(demo_factory ${PROJECT_NAME})

add_executable(demo_inheritance demo_inheritance.cpp)
target_link_libraries(demo_inheritance ${PROJECT_NAME})

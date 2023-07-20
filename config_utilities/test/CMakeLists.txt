catkin_add_gtest(
  test_${PROJECT_NAME}
  main.cpp
  src/utils.cpp
  tests/traits.cpp
)
target_include_directories(test_${PROJECT_NAME} PUBLIC include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} yaml-cpp)

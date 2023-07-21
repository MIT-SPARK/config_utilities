catkin_add_gtest(
  test_${PROJECT_NAME}
  main.cpp
  src/utils.cpp
  src/default_configs.cpp
  tests/traits.cpp
  tests/validity_checks.cpp
  tests/namespacing.cpp
  tests/yaml_parsing.cpp
)
target_include_directories(test_${PROJECT_NAME} PUBLIC include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

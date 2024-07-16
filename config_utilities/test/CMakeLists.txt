find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/default_config.cpp
  src/utils.cpp
  tests/asl_formatter.cpp
  tests/config_arrays.cpp
  tests/config_maps.cpp
  tests/conversions.cpp
  tests/enums.cpp
  tests/factory.cpp
  tests/inheritance.cpp
  tests/missing_fields.cpp
  tests/namespacing.cpp
  tests/path.cpp
  tests/string_utils.cpp
  tests/subconfigs.cpp
  tests/traits.cpp
  tests/update.cpp
  tests/validity_checks.cpp
  tests/virtual_config.cpp
  tests/yaml_parsing.cpp)
target_include_directories(test_${PROJECT_NAME} PRIVATE include)
target_link_libraries(test_${PROJECT_NAME} PRIVATE ${PROJECT_NAME}
                                                   GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})

if(${CONFIG_UTILS_INSTALL_TESTS})
  # note that ros uses libdir to handle finding executables by package, but this
  # isn't an ideal install location normally
  install(TARGETS test_${PROJECT_NAME}
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
endif()

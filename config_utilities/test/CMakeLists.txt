find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

add_library(test_${PROJECT_NAME}_plugins src/external_plugins.cpp)
target_include_directories(test_${PROJECT_NAME}_plugins PRIVATE include)
target_link_libraries(test_${PROJECT_NAME}_plugins ${PROJECT_NAME})
target_compile_options(test_${PROJECT_NAME}_plugins PRIVATE -fvisibility=hidden)

# Add new test resources here to ensure they get copied to the approriate build
# dir.
set(TEST_RESOURCES resources/foo.yaml resources/bar.yaml resources/invalid.yaml)

list(TRANSFORM TEST_RESOURCES PREPEND "${CMAKE_CURRENT_BINARY_DIR}/"
                                      OUTPUT_VARIABLE TEST_RESOURCE_BUILD_PATHS)
list(TRANSFORM TEST_RESOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/"
                                      OUTPUT_VARIABLE TEST_RESOURCE_SRC_PATHS)

add_custom_command(
  OUTPUT ${TEST_RESOURCE_BUILD_PATHS}
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${CMAKE_CURRENT_BINARY_DIR}/resources
  COMMAND ${CMAKE_COMMAND} -E copy ${TEST_RESOURCE_SRC_PATHS}
          ${CMAKE_CURRENT_BINARY_DIR}/resources/
  COMMAND_EXPAND_LISTS
  COMMENT "Copy test resources to build directory"
  DEPENDS ${TEST_RESOURCE_SRC_PATHS})
add_custom_target(
  copy_test_resources
  COMMENT "Target for test resources"
  DEPENDS ${TEST_RESOURCE_BUILD_PATHS})

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/cli_args.cpp
  src/default_config.cpp
  src/introspection_utils.cpp
  src/utils.cpp
  tests/asl_formatter.cpp
  tests/commandline.cpp
  tests/config_arrays.cpp
  tests/config_maps.cpp
  tests/conversions.cpp
  tests/dynamic_config.cpp
  tests/enums.cpp
  tests/external_registry.cpp
  tests/factory.cpp
  tests/getters.cpp
  tests/field_input_info.cpp
  tests/inheritance.cpp
  tests/introspection.cpp
  tests/missing_fields.cpp
  tests/namespacing.cpp
  tests/path.cpp
  tests/string_utils.cpp
  tests/subconfigs.cpp
  tests/substitutions.cpp
  tests/traits.cpp
  tests/update.cpp
  tests/validity_checks.cpp
  tests/virtual_config.cpp
  tests/yaml_parsing.cpp
  tests/yaml_utils.cpp
  )
add_dependencies(test_${PROJECT_NAME} copy_test_resources)
target_include_directories(test_${PROJECT_NAME} PRIVATE include)
target_link_libraries(test_${PROJECT_NAME} PRIVATE ${PROJECT_NAME}
                                                   GTest::gtest_main)
target_compile_options(test_${PROJECT_NAME} PRIVATE -Wno-unused-but-set-variable)
gtest_add_tests(TARGET test_${PROJECT_NAME})

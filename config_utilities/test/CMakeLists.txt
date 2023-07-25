include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG b796f7d44681514f58a683a3a71ff17c94edb0c1 # v1.13.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/default_config.cpp
  src/utils.cpp
  tests/asl_formatter.cpp
  tests/conversions.cpp
  tests/factory.cpp
  tests/globals.cpp
  tests/inheritance.cpp
  tests/namespacing.cpp
  tests/string_utils.cpp
  tests/traits.cpp
  tests/validity_checks.cpp
  tests/virtual_config.cpp
  tests/yaml_parsing.cpp
)
target_include_directories(test_${PROJECT_NAME} PRIVATE include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)

if(${CONFIG_UTILS_INSTALL_TESTS})
  # note that ros uses libdir to handle finding executables by package, but this isn't
  # an ideal install location normally
  install(TARGETS test_${PROJECT_NAME}
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()

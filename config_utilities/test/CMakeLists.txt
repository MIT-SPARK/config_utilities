include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  src/utils.cpp
  src/default_config.cpp
  tests/traits.cpp
  tests/validity_checks.cpp
  tests/namespacing.cpp
  tests/yaml_parsing.cpp
  tests/asl_formatter.cpp
  tests/inheritance.cpp
)
target_include_directories(test_${PROJECT_NAME} PRIVATE include)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_${PROJECT_NAME})

if(${CONFIG_UTILS_INSTALL_TESTS})
  # note that ros uses libdir to handle finding executables by package, but this isn't
  # an ideal install location normally
  install(TARGETS test_${PROJECT_NAME}
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG b796f7d44681514f58a683a3a71ff17c94edb0c1 # v1.13.0
  INSTALL_COMMAND "")

# note that FetchContent_MakeAvailable automatically installs any added
# fetch-content project, see:
# https://stackoverflow.com/questions/65527126/disable-install-for-fetchcontent
FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  # We want to build static gtest libraries to avoid issues, so we have to cache
  # and reset the relevant variables, see:
  # https://stackoverflow.com/questions/62101576/using-fetchcontent-declare-together-with-cmake-args
  set(BUILD_SHARED_LIBS_THIS ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS
      OFF
      CACHE INTERNAL "Build static libraries")
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR}
                   EXCLUDE_FROM_ALL)
  set(BUILD_SHARED_LIBS
      ${BUILD_SHARED_LIBS_THIS}
      CACHE BOOL "Build SHARED libraries")
endif()

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
  tests/globals.cpp
  tests/inheritance.cpp
  tests/namespacing.cpp
  tests/path.cpp
  tests/string_utils.cpp
  tests/subconfigs.cpp
  tests/traits.cpp
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

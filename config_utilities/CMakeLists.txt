cmake_minimum_required(VERSION 3.14)
project(config_utilities VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(OptionalPackage)

option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(CONFIG_UTILS_ENABLE_ROS "Export roscpp and build related code" ON)
option(CONFIG_UTILS_ENABLE_EIGEN "Export Eigen and build related code" ON)
option(CONFIG_UTILS_ENABLE_GLOG "Export glog and build related code" ON)
option(CONFIG_UTILS_BUILD_TESTS "Build unit tests" ON)
option(CONFIG_UTILS_BUILD_DEMOS "Build demo executables" ON)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_optional(roscpp CONFIG_UTILS_ENABLE_ROS)
find_optional(Eigen3 CONFIG_UTILS_ENABLE_EIGEN)
find_optional_pkgcfg(libglog CONFIG_UTILS_ENABLE_GLOG)

add_library(
  ${PROJECT_NAME}
  src/asl_formatter.cpp
  src/commandline.cpp
  src/config_context.cpp # global singleton
  src/context.cpp # parsing
  src/conversions.cpp
  src/external_registry.cpp
  src/factory.cpp
  src/formatter.cpp
  src/logger.cpp
  src/log_to_stdout.cpp
  src/meta_data.cpp
  src/namespacing.cpp
  src/path.cpp
  src/settings.cpp
  src/string_utils.cpp
  src/substitutions.cpp
  src/substitution_parsers.cpp
  src/validation.cpp
  src/visitor.cpp
  src/yaml_parser.cpp
  src/yaml_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC yaml-cpp
  PRIVATE Boost::filesystem Boost::system Boost::boost ${CMAKE_DL_LIBS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(config_utilities::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

add_executable(composite-configs app/composite_configs.cpp)
target_link_libraries(composite-configs ${PROJECT_NAME})

if(ENABLE_roscpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${roscpp_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PUBLIC ${roscpp_INCLUDE_DIRS})
endif()

if(ENABLE_Eigen3)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
endif()

if(ENABLE_libglog)
  target_link_libraries(${PROJECT_NAME} PUBLIC PkgConfig::libglog)
endif()

message(STATUS "Eigen features enabled: ${ENABLE_Eigen3}")
message(STATUS "ROS features enabled:   ${ENABLE_roscpp}")
message(STATUS "glog features enabled:  ${ENABLE_libglog}")

if(CONFIG_UTILS_BUILD_DEMOS)
  add_subdirectory(demos)
endif()

if(CONFIG_UTILS_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(test)
endif()

include(HandleInstall)

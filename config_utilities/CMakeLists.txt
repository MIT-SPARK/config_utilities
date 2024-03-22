cmake_minimum_required(VERSION 3.14)
project(config_utilities VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(OptionalPackage)

option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(CONFIG_UTILS_ENABLE_ROS "Export roscpp and build related code" ON)
option(CONFIG_UTILS_ENABLE_EIGEN "Export Eigen and build related code" ON)
option(CONFIG_UTILS_ENABLE_GLOG "Export glog and build related code" ON)
option(CONFIG_UTILS_BUILD_TESTS "Build unit tests" ON)
option(CONFIG_UTILS_INSTALL_TESTS "Install test executable" ON)
option(CONFIG_UTILS_BUILD_DEMOS "Build demo executables" ON)

find_package(yaml-cpp REQUIRED)
FIND_OPTIONAL(roscpp CONFIG_UTILS_ENABLE_ROS)
FIND_OPTIONAL(Eigen3 CONFIG_UTILS_ENABLE_EIGEN)
FIND_OPTIONAL_PKGCFG(libglog CONFIG_UTILS_ENABLE_GLOG)

add_library(
  ${PROJECT_NAME}
  src/asl_formatter.cpp
  src/conversions.cpp
  src/formatter.cpp
  src/logger.cpp
  src/meta_data.cpp
  src/namespacing.cpp
  src/path.cpp
  src/settings.cpp
  src/string_utils.cpp
  src/validation.cpp
  src/visitor.cpp
  src/yaml_parser.cpp
  src/yaml_utils.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(config_utilities::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

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
  enable_testing()
  add_subdirectory(test)
endif()

include(HandleInstall)

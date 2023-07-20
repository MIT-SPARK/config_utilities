cmake_minimum_required(VERSION 3.10)
project(config_utilities VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(yaml-cpp REQUIRED)

option(CONFIG_UTILS_ENABLE_ROS "Build ROS related code" ON)
option(CONFIG_UTILS_ENABLE_EIGEN "Build Eigen related code" ON)
option(CONFIG_UTILS_ENABLE_GLOG "Build glog related code" ON)
option(CONFIG_UTILS_BUILD_TESTS "Build unit tests" ON)
option(CONFIG_UTILS_BUILD_DEMOS "Build demo executables" ON)
# TODO(nathan) executable install options

add_library(
  ${PROJECT_NAME}
  src/meta_data.cpp
  src/visitor.cpp
  src/yaml_parser.cpp
  src/settings.cpp
  src/printing.cpp
  src/asl_formatter.cpp
  src/string_utils.cpp
  src/yaml_utils.cpp
  src/formatter.cpp
  src/globals.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

# TODO(nathan) refactor
if (CONFIG_UTILS_ENABLE_ROS)
  find_package(roscpp)
  if (roscpp_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${roscpp_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${roscpp_INCLUDE_DIRS})
  else()
    message(WARNING "ROS was not found. Consider disabling")
    set(CONFIG_UTILS_ENABLE_ROS OFF)
  endif()
endif(CONFIG_UTILS_ENABLE_ROS)

if (CONFIG_UTILS_ENABLE_EIGEN)
  find_package(Eigen3)
  if (Eigen3_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
  else()
    message(WARNING "Eigen3 was not found. Consider disabling")
    set(CONFIG_UTILS_ENABLE_EIGEN OFF)
  endif()
endif(CONFIG_UTILS_ENABLE_EIGEN)

if(CONFIG_UTILS_BUILD_DEMOS)
  add_subdirectory(demos)
endif(CONFIG_UTILS_BUILD_DEMOS)

if(CONFIG_UTILS_BUILD_TESTS)
  add_subdirectory(tests)
endif(CONFIG_UTILS_BUILD_TESTS)

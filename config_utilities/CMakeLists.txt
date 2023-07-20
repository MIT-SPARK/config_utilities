cmake_minimum_required(VERSION 3.10)
project(config_utilities VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(yaml-cpp REQUIRED)

option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(CONFIG_UTILS_ENABLE_ROS "Export roscpp and build related code" ON)
option(CONFIG_UTILS_ENABLE_EIGEN "Export Eigen and build related code" ON)
option(CONFIG_UTILS_ENABLE_GLOG "Export glog and build related code" ON)
option(CONFIG_UTILS_BUILD_TESTS "Build unit tests" ON)
option(CONFIG_UTILS_INSTALL_TESTS "Install test executable" ON)
option(CONFIG_UTILS_BUILD_DEMOS "Build demo executables" ON)
option(CONFIG_UTILS_INSTALL_DEMOS "Install demo executable" ON)

macro(FIND_OPTIONAL package_name package_option)
  find_package(${package_name})
  if(${package_name}_FOUND)
    set(ENABLE_${package_name} TRUE)
  else()
    set(ENABLE_${package_name} FALSE)
    if(${package_option})
      message(WARNING "${package_name} not found. Consider disabling")
    endif()
  endif()
endmacro()

find_optional(roscpp CONFIG_UTILS_ENABLE_ROS)
find_optional(Eigen3 CONFIG_UTILS_ENABLE_EIGEN)

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
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)

if(ENABLE_roscpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${roscpp_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PUBLIC ${roscpp_INCLUDE_DIRS})
endif()

if(ENABLE_Eigen3)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
endif()

if(CONFIG_UTILS_BUILD_DEMOS)
  add_subdirectory(demos)
endif()

if(CONFIG_UTILS_BUILD_TESTS)
  add_subdirectory(tests)
endif()

add_library(config_utilities::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include(GNUInstallDirs)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT config_utilities-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT config_utilities-targets
  FILE config_utilitiesTargets.cmake
  NAMESPACE config_utilities::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/config_utilitiesConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)
write_basic_package_version_file(
  config_utilitiesConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)

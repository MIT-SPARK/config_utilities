cmake_minimum_required(VERSION 3.22.1)
project(config_utilities VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(CONFIG_UTILS_BUILD_DEMOS "Build demo executables" ON)

find_package(yaml-cpp REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 QUIET)
find_package(glog QUIET)
find_package(nlohmann_json QUIET)

add_library(
  ${PROJECT_NAME}
  src/asl_formatter.cpp
  src/commandline.cpp
  src/config_context.cpp # global singleton
  src/context.cpp # parsing
  src/conversions.cpp
  src/dynamic_config.cpp
  src/external_registry.cpp
  src/factory.cpp
  src/formatter.cpp
  src/field_input_info.cpp
  src/introspection.cpp
  src/log_to_stdout.cpp
  src/log_to_glog.cpp
  src/logger.cpp
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
  src/yaml_utils.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Set up main library linkage
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC yaml-cpp
  PRIVATE Boost::filesystem Boost::system Boost::boost ${CMAKE_DL_LIBS})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(config_utilities::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
if(Eigen3_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
endif()
if(glog_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_UTILS_ENABLE_GLOG_LOGGING)
endif()
if(nlohmann_json_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_UTILS_ENABLE_JSON)
endif()

add_executable(composite-configs app/composite_configs.cpp)
target_link_libraries(composite-configs ${PROJECT_NAME})

if(CONFIG_UTILS_BUILD_DEMOS)
  add_subdirectory(demos)
endif()

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

# Install targets and headers
include(GNUInstallDirs)
install(
  TARGETS ${PROJECT_NAME} composite-configs
  EXPORT config_utilities-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY introspection_viewer/ DESTINATION ${CMAKE_INSTALL_BINDIR})
install(PROGRAMS introspection_viewer/introspection_viewer.py
        RENAME config-utilities-viewer
        DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT config_utilities-targets
  FILE config_utilitiesTargets.cmake
  NAMESPACE config_utilities::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)

# Install packaging
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

# Exposes this package to ament for ros2 run and other utilities to work
find_package(ament_cmake_core QUIET)
if (${ament_cmake_core_FOUND}) 
  ament_package()
endif()

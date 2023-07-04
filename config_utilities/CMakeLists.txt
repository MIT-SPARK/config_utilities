cmake_minimum_required(VERSION 3.0.2)
project(config_utilities)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall)
find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(yaml-cpp REQUIRED)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/yaml_parser.cpp
        src/settings.cpp
        src/asl_formatter.cpp
        src/string_utils.cpp
        )
target_link_libraries(${PROJECT_NAME} yaml-cpp)

###############
# Executables #
###############

cs_add_executable(demo_config
        demos/config.cpp
        )
target_link_libraries(demo_config
${PROJECT_NAME}
yaml-cpp)

cs_add_executable(demo_factory
        demos/factory.cpp
        )
target_link_libraries(demo_config
${PROJECT_NAME}
yaml-cpp)

##########
# Export #
##########

cs_install()
cs_export()

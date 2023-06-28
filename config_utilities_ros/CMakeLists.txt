cmake_minimum_required(VERSION 3.0.2)
project(config_utilities_ros)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall)
find_package(catkin_simple REQUIRED)
catkin_simple()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/ros_parser.cpp
        )
target_link_libraries(${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()

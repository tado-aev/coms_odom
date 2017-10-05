cmake_minimum_required(VERSION 2.8.3)
project(coms_odom)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    coms_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES coms_odom
    CATKIN_DEPENDS roscpp coms_msgs sensor_msgs nav_msgs tf2_ros
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    )

add_library(
    coms_odom
    src/coms_odom.cpp
    include/coms_odom/coms_odom.h
    )

add_executable(
    coms_odom_node
    src/coms_odom_node.cpp
    )

target_link_libraries(coms_odom_node
    coms_odom
    ${catkin_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    )

install(TARGETS coms_odom_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

cmake_minimum_required(VERSION 2.8.3)
project(elektron_rapps)

find_package(Rapp REQUIRED)
find_package(Boost COMPONENTS system random REQUIRED)
find_package(RappRobots REQUIRED)
include_directories(${RAPP_INCLUDE_DIRS}/rapp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${Rapp_DIR})
include(RappMacros)

find_library(PTHREAD NAMES libpthread.so.0 )

setup_rapp()

create_rapp(
    NAME object_search
    SRCS src/object_search.cpp
    LIBS ${PTHREAD} ${Boost_LIBRARIES}
 #   [FILES file1, file2, ...]
)


create_rapp(
    NAME elektron_hazard
    SRCS src/hazard_detection.cpp
    LIBS ${PTHREAD} ${Boost_LIBRARIES}
 #   [FILES file1, file2, ...]
)
create_rapp(
    NAME vis_loc
    SRCS src/vis_loc.cpp
    LIBS ${PTHREAD} ${Boost_LIBRARIES}
 #   [FILES file1, file2, ...]
)
# install(TARGETS elektron_nav_test
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



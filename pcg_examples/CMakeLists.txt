cmake_minimum_required(VERSION 2.8.3)
project(pcg_examples)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcg_examples
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(
    DIRECTORY config meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE)
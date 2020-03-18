cmake_minimum_required(VERSION 2.8.3)
project(pcg_gazebo)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

#############
## Install ##
#############

install(
  DIRECTORY templates
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE)

install(
  DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ${PROJECT_NAME}-extras.cmake* EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  
endif()
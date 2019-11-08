cmake_minimum_required(VERSION 2.8.3)
project(pcg_examples)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcg_examples
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(PROGRAMS
    scripts/generate_gazebo_models_from_templates
    scripts/generate_pcg_world_files
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch config meshes models templates worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE)
cmake_minimum_required(VERSION 2.8.3)
project(pcg_gazebo)

find_package(catkin REQUIRED COMPONENTS pcg_libraries)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

#############
## Install ##
#############

install(PROGRAMS
    scripts/generate_model_description
    scripts/sdf2urdf
    scripts/urdf2sdf
    scripts/gazebo2tf
    scripts/generate_pcg_world
    scripts/spawn_sdf_model
    scripts/run_model_factory
    scripts/generate_occupancy_map
    scripts/start_gazebo_world
    scripts/view_mesh
    scripts/view_gazebo_model
    scripts/list_gazebo_models
    scripts/urdflint
    scripts/sdflint
    scripts/process_jinja_template
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE)

install(
  DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ${PROJECT_NAME}-extras.cmake* EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/gazebo2tf.launch)
  roslaunch_add_file_check(launch/generate_model_description.launch)
  roslaunch_add_file_check(launch/generate_pcg_world.launch)
#   roslaunch_add_file_check(launch/start_generated_gazebo_world.launch)
endif()
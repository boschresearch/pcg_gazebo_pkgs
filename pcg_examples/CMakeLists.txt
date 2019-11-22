cmake_minimum_required(VERSION 2.8.3)
project(pcg_examples)

find_package(catkin REQUIRED COMPONENTS 
    gazebo_ros
    pcg_gazebo)

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
    scripts/gen_model_from_factory_config
    scripts/gen_model_from_template    
    scripts/launch_pcg_world
    scripts/spawn_model_from_factory_config
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch config meshes templates 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(rosunit REQUIRED)  

    foreach(T
        test/test_model_factory_examples.py
        test/test_template_examples.py
        test/test_world_gen_examples.py)
        catkin_add_nosetests(${T})
    endforeach()
endif()
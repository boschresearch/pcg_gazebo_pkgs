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

# Use the PCG world configurations to generate the Gazebo 
# worlds with object pose randomization
file(GLOB WORLD_CONFIGS 
    ${CMAKE_CURRENT_LIST_DIR}/config/world_generator/worlds/*.yml)

foreach(WORLD_CONFIG ${WORLD_CONFIGS})
    get_filename_component(WORLD_NAME "${WORLD_CONFIG}" NAME_WE)                

    pcg_generate_world_from_config(
        CONFIG_FILENAME ${WORLD_CONFIG}
        OUTPUT_WORLD_FILENAME ${CMAKE_CURRENT_LIST_DIR}/worlds/${WORLD_NAME}.world
    )
endforeach()

# Use the PCG model group configuration to generate
# static Gazebo models
file(GLOB MODEL_CONFIGS 
    ${CMAKE_CURRENT_LIST_DIR}/templates/models/*.sdf.jinja)

foreach(MODEL_CONFIG ${MODEL_CONFIGS})
    get_filename_component(MODEL_NAME "${MODEL_CONFIG}" NAME_WE)                

    pcg_convert_jinja_to_gazebo_model(
        INPUT_TEMPLATE_FILENAME ${MODEL_CONFIG}
        MODEL_NAME ${MODEL_NAME}
        OUTPUT_MODEL_DIR ${CMAKE_CURRENT_LIST_DIR}/models
        SDF_VERSION "1.6"
        MODEL_VERSION 1.0)
endforeach()

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
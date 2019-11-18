# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
function(pcg_generate_world_from_config)
    set(ONE_VALUE_KEYWORDS 
        CONFIG_FILENAME
        OUTPUT_WORLD_FILENAME
        VERBOSE
        WITH_SUN
        WITH_GROUND_PLANE
        PHYSICS_ENGINE
        OVERWRITE)

    set(MULTI_VALUE_KEYWORDS "")

    cmake_parse_arguments(ARG "${OPTIONS}" "${ONE_VALUE_KEYWORDS}" "${MULTI_VALUE_KEYWORDS}" ${ARGN})

    if(NOT DEFINED ARG_CONFIG_FILENAME)    
        message(SEND_ERROR "Input configuration file was not provided")
    endif()

    if(NOT DEFINED ARG_OUTPUT_WORLD_FILENAME)
        string(TIMESTAMP CUR_TIMESTAMP "%Y-%m-%d_%H_%M")
        set(ARG_OUTPUT_WORLD_FILENAME "/tmp/pcg_$ENV{USERNAME}_${CUR_TIMESTAMP}.world")
    endif()

    if(NOT DEFINED ARG_VERBOSE)
        set(ARG_VERBOSE false)
    endif()

    if(ARG_VERBOSE)
        set(VERBOSE_OPT "--verbose")
    else()  
        set(VERBOSE_OPT "")
    endif()

    if(NOT DEFINED ARG_WITH_SUN)
        set(ARG_WITH_SUN false)
    endif()

    if(ARG_WITH_SUN)
        set(WITH_SUN_OPT "--with-sun")
    else()  
        set(WITH_SUN_OPT "")
    endif()

    if(NOT DEFINED ARG_WITH_GROUND_PLANE)
        set(ARG_WITH_GROUND_PLANE false)
    endif()

    if(ARG_WITH_GROUND_PLANE)
        set(WITH_GROUND_PLANE_OPT "--with-ground-plane")
    else()  
        set(WITH_GROUND_PLANE_OPT "")
    endif()

    if(NOT DEFINED ARG_PHYSICS_ENGINE)
        set(ARG_PHYSICS_ENGINE "ode")
    endif()

    if(NOT DEFINED ARG_OVERWRITE)
        set(ARG_OVERWRITE false)
    endif()

    if(EXISTS ${ARG_OUTPUT_WORLD_FILENAME} AND NOT ${ARG_OVERWRITE})
        message(STATUS "Output world file \"${ARG_OUTPUT_WORLD_FILENAME}\" already exists. To generate the world again, delete the file or set the OVERWRITE to true")
    else()
        string(REGEX REPLACE "/" "_" WORLD_GEN_STR "WORLD_GEN_STR_${ARG_OUTPUT_WORLD_FILENAME}")

        set(WORLD_GEN_FAKE "${CMAKE_CURRENT_BINARY_DIR}/${WORLD_GEN_STR}_pcg.world.fake")

        if(EXISTS ${WORLD_GEN_FAKE})
            message(FATAL_ERROR "File \"${WORLD_GEN_FAKE}\" found, this should never be created, remove!")
        endif()    

        add_custom_target(
            ${WORLD_GEN_STR} ALL
            DEPENDS ${WORLD_GEN_FAKE})

        add_custom_command(
            OUTPUT 
                ${WORLD_GEN_FAKE}
                ${ARG_OUTPUT_WORLD_FILENAME}
            COMMAND rosrun pcg_gazebo generate_pcg_world 
                --config-file ${ARG_CONFIG_FILENAME} 
                --output-world-file ${ARG_OUTPUT_WORLD_FILENAME}
                --physics ${ARG_PHYSICS_ENGINE}
                ${VERBOSE_OPT} ${WITH_SUN_OPT} ${WITH_GROUND_PLANE_OPT}
        )

        if(EXISTS ${ARG_OUTPUT_WORLD_FILENAME})
            message(STATUS "World file \"${ARG_OUTPUT_WORLD_FILENAME}\" successfully generated")
        else()
            message(WARNING "World file \"${ARG_OUTPUT_WORLD_FILENAME}\" COULD NOT be generated")
        endif()
    
        unset(WORLD_GEN_FAKE)        
        
    endif()

endfunction()
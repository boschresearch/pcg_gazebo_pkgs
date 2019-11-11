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
function(pcg_convert_jinja_to_sdf)
    set(one_value_keywords INPUT_TEMPLATE_FILENAME 
        OUTPUT_SDF_FILENAME 
        OUTPUT_SDF_DIR 
        JINJA_INPUT_PARAMETERS_YAML_FILE
        GENERATE_URDF
        OUTPUT_URDF_FILENAME
        OUTPUT_URDF_DIR
        MERGE_NESTED_MODELS)
    set(multi_value_keywords JINJA_INPUT_PARAMETERS)

    cmake_parse_arguments(ARG "${options}" "${one_value_keywords}" "${multi_value_keywords}" ${ARGN})

    if(NOT DEFINED ARG_INPUT_TEMPLATE_FILENAME)
        message(SEND_ERROR "Input template Jinja template has not been provided")
    else()
        message(STATUS "Processing template Jinja file: ${ARG_INPUT_TEMPLATE_FILENAME}")
    endif()

    if(NOT DEFINED ARG_OUTPUT_SDF_DIR)
        set(ARG_OUTPUT_SDF_DIR ${CMAKE_CURRENT_LIST_DIR})
    endif()

    message(STATUS "Output directory for generated SDF file: ${ARG_OUTPUT_SDF_DIR}")

    if(NOT DEFINED ARG_OUTPUT_URDF_DIR)
        set(ARG_OUTPUT_URDF_DIR ${CMAKE_CURRENT_LIST_DIR})
    endif()

    if(NOT DEFINED ARG_OUTPUT_SDF_FILENAME)        
        get_filename_component(INPUT_TEMPLATE_WITHOUT_EXT "${ARG_INPUT_TEMPLATE_FILENAME}" NAME_WE)                
        set(ARG_OUTPUT_SDF_FILENAME "${INPUT_TEMPLATE_WITHOUT_EXT}.sdf")        
    endif()

    message(STATUS "Output SDF filename: ${ARG_OUTPUT_SDF_FILENAME}")

    if(NOT DEFINED ARG_GENERATE_URDF)
        set(ARG_GENERATE_URDF false)
    endif()

    if(NOT DEFINED ARG_OUTPUT_URDF_FILENAME)
        get_filename_component(INPUT_TEMPLATE_WITHOUT_EXT "${ARG_INPUT_TEMPLATE_FILENAME}" NAME_WE)
        set(ARG_OUTPUT_URDF_FILENAME "${INPUT_TEMPLATE_WITHOUT_EXT}.urdf")        
    endif()

    if(DEFINED ARG_JINJA_INPUT_PARAMETERS_YAML_FILE)
        set(TEMPLATE_PARAMETERS_FILENAME "--param-file ${ARG_JINJA_INPUT_PARAMETERS_YAML_FILE}")
        message(STATUS "Template parameters YAML file provided: ${ARG_JINJA_INPUT_PARAMETERS_YAML_FILE}")
    else()
        set(TEMPLATE_PARAMETERS_FILENAME "")
        message(STATUS "No template parameters YAML file provided")
    endif()

    if(NOT DEFINED ARG_MERGE_NESTED_MODELS)
        set(ARG_MERGE_NESTED_MODELS false)
    endif()

    if(ARG_MERGE_NESTED_MODELS)
        set(MERGE_NESTED_MODELS_OPT "--merge-nested-models")
    else()
        set(MERGE_NESTED_MODELS_OPT "")
    endif()
        
    string(REGEX REPLACE "/" "_" MODEL_SDF_TARGET_STR "MODEL_SDF_TARGET_${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}")

    set(MODEL_SDF_FAKE "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_SDF_TARGET_STR}_model.sdf.fake")

    if(EXISTS ${MODEL_SDF_FAKE})
        message(FATAL_ERROR "File \"${MODEL_SDF_FAKE}\" found, this should never be created, remove!")
    endif()    

    add_custom_target(
        ${MODEL_SDF_TARGET_STR} ALL
        DEPENDS ${MODEL_SDF_FAKE})

    add_custom_command(
        OUTPUT 
            ${MODEL_SDF_FAKE}
            ${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}
        COMMAND rosrun pcg_gazebo process_jinja_template 
            --input-template ${ARG_INPUT_TEMPLATE_FILENAME} 
            --output-filename ${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}                 
            ${ARG_JINJA_INPUT_PARAMETERS} ${TEMPLATE_PARAMETERS_FILENAME} --sdf ${MERGE_NESTED_MODELS_OPT}
    )

    unset(MODEL_SDF_FAKE)        
            
    message(STATUS "Output generated file: ${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}")

    if(ARG_GENERATE_URDF)
        message(STATUS "Output converted URDF file: ${ARG_OUTPUT_URDF_DIR}/${ARG_OUTPUT_URDF_FILENAME}")

        string(REGEX REPLACE "/" "_" MODEL_URDF_TARGET_STR "MODEL_URDF_TARGET_${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}")

        set(MODEL_URDF_FAKE "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_SDF_TARGET_STR}_model.urdf.fake")

        if(EXISTS ${MODEL_URDF_FAKE})
            message(FATAL_ERROR "File \"${MODEL_URDF_FAKE}\" found, this should never be created, remove!")
        endif()    

        add_custom_target(
            ${MODEL_URDF_TARGET_STR} ALL
            DEPENDS ${MODEL_URDF_FAKE})

        add_custom_command(
            OUTPUT
                ${MODEL_URDF_FAKE}
                ${ARG_OUTPUT_URDF_DIR}/${ARG_OUTPUT_URDF_FILENAME}
            COMMAND rosrun pcg_gazebo sdf2urdf
                --filename ${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME}
                --output-filename ${ARG_OUTPUT_URDF_DIR}/${ARG_OUTPUT_URDF_FILENAME}
            DEPENDS 
                ${ARG_OUTPUT_SDF_DIR}/${ARG_OUTPUT_SDF_FILENAME})
    endif()
endfunction()
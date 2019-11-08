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
function(pcg_convert_jinja_to_gazebo_model)
    set(one_value_keywords 
        INPUT_TEMPLATE_FILENAME 
        MODEL_NAME
        OUTPUT_MODEL_DIR 
        SDF_VERSION
        MODEL_VERSION
        AUTHOR_NAME
        AUTHOR_EMAIL                
        DESCRIPTION
        GENERATE_URDF
        JINJA_INPUT_PARAMETERS_YAML_FILE)
    set(multi_value_keywords JINJA_INPUT_PARAMETERS)

    cmake_parse_arguments(ARG "${options}" "${one_value_keywords}" "${multi_value_keywords}" ${ARGN})

    if(NOT DEFINED ARG_INPUT_TEMPLATE_FILENAME)
        message(SEND_ERROR "Input template Jinja template has not been provided")
    else()
        message(STATUS "Processing template Jinja file: ${ARG_INPUT_TEMPLATE_FILENAME}")
    endif()

    if(NOT DEFINED ARG_MODEL_NAME)
        message(SEND_ERROR "Model name was not provided")
    endif()

    if(NOT DEFINED ARG_OUTPUT_MODEL_DIR)
        set(ARG_OUTPUT_MODEL_DIR "$ENV{HOME}/.gazebo/models")
    endif()

    if(NOT IS_DIRECTORY ${ARG_OUTPUT_MODEL_DIR})
        message(SEND_ERROR "Output model directory does not exist, dir=${ARG_OUTPUT_MODEL_DIR}")
    else()
        message(STATUS "Model <${ARG_MODEL_NAME}> to be stored in directory ${ARG_OUTPUT_MODEL_DIR}")
    endif() 

    if(NOT DEFINED ARG_AUTHOR_NAME)
        execute_process(COMMAND bash -c "whoami" OUTPUT_VARIABLE ARG_AUTHOR_NAME)
        # Remove break line
        string(REGEX REPLACE "\n$" "" ARG_AUTHOR_NAME "${ARG_AUTHOR_NAME}")
    endif()

    message(STATUS "Model author: ${ARG_AUTHOR_NAME}")

    if(NOT DEFINED ARG_AUTHOR_EMAIL)
        set(ARG_AUTHOR_EMAIL "${ARG_AUTHOR_NAME}@email.com")
        # Remove break line
        string(REGEX REPLACE "\n$" "" ARG_AUTHOR_EMAIL "${ARG_AUTHOR_EMAIL}")
    endif()

    message(STATUS "Model e-mail: ${ARG_AUTHOR_EMAIL}")

    if(NOT DEFINED ARG_MODEL_VERSION)
        set(ARG_MODEL_VERSION "1.0")
    endif()

    message(STATUS "Model version: ${ARG_MODEL_VERSION}")

    if(NOT DEFINED ARG_SDF_VERSION)
        set(ARG_SDF_VERSION "1.6")
    endif()

    message(STATUS "Model SDF version: ${ARG_SDF_VERSION}")

    if(NOT DEFINED ARG_DESCRIPTION)
        set(ARG_DESCRIPTION "")
    endif()

    if(NOT DEFINED ARG_GENERATE_URDF)
        set(ARG_GENERATE_URDF false)
    endif()

    string(COMPARE EQUAL "${ARG_DESCRIPTION}" "" HAS_DESCRIPTION)
    if(HAS_DESCRIPTION)
        message(STATUS "has no description")
    endif()

    # Create model directory        
    message(STATUS "Creating ${ARG_OUTPUT_MODEL_DIR}/${ARG_MODEL_NAME}")
    file(MAKE_DIRECTORY ${ARG_OUTPUT_MODEL_DIR}/${ARG_MODEL_NAME})

    execute_process(
        COMMAND bash -c "rospack find pcg_libraries" 
        OUTPUT_VARIABLE MODEL_CONFIG_TEMPLATE_DIR)

    # Remove break line
    string(REGEX REPLACE "\n$" "" MODEL_CONFIG_TEMPLATE_DIR "${MODEL_CONFIG_TEMPLATE_DIR}")
    
    execute_process(COMMAND rosrun pcg_gazebo process_jinja_template 
        --input-template ${MODEL_CONFIG_TEMPLATE_DIR}/sdf/model.config.jinja 
        --output-filename ${ARG_OUTPUT_MODEL_DIR}/${ARG_MODEL_NAME}/model.config 
        --param=model_name=${ARG_MODEL_NAME} 
        --param=version=${ARG_MODEL_VERSION} 
        --param=sdf_version=${ARG_SDF_VERSION} 
        --param=author_name=${ARG_AUTHOR_NAME} 
        --param=author_email=${ARG_AUTHOR_EMAIL} 
        --param=sdf_filename=model.sdf --sdf-config)

    pcg_convert_jinja_to_sdf(
        INPUT_TEMPLATE_FILENAME ${ARG_INPUT_TEMPLATE_FILENAME}
        OUTPUT_SDF_DIR ${ARG_OUTPUT_MODEL_DIR}/${ARG_MODEL_NAME}
        OUTPUT_SDF_FILENAME model.sdf
        JINJA_INPUT_PARAMETERS_YAML_FILE ${ARG_JINJA_INPUT_PARAMETERS_YAML_FILE}
        JINJA_INPUT_PARAMETERS ${ARG_JINJA_INPUT_PARAMETERS}
        GENERATE_URDF ${ARG_GENERATE_URDF}
        OUTPUT_URDF_DIR ${ARG_OUTPUT_MODEL_DIR}/${ARG_MODEL_NAME}
        OUTPUT_URDF_FILENAME model.urdf)
endfunction()
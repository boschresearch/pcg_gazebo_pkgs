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
function(pcg_generate_model_from_config)
    set(ONE_VALUE_KEYWORDS 
        CONFIG_FILENAME
        OUTPUT_MODEL_DIR
        OVERWRITE)

    set(MULTI_VALUE_KEYWORDS "")

    cmake_parse_arguments(ARG "${OPTIONS}" "${ONE_VALUE_KEYWORDS}" "${MULTI_VALUE_KEYWORDS}" ${ARGN})

    if(NOT DEFINED ARG_CONFIG_FILENAME)    
        message(SEND_ERROR "Input configuration file was not provided")
    endif()

    if(NOT DEFINED ARG_OUTPUT_MODEL_DIR)
        set(ARG_OUTPUT_MODEL_DIR "$ENV{HOME}/.gazebo/models")
    endif()

    if(NOT DEFINED ARG_OVERWRITE)
        set(ARG_OVERWRITE false)
    endif()

    if(ARG_OVERWRITE)
        set(OVERWRITE_MODELS_OPT "--overwrite")
    else()  
        set(OVERWRITE_MODELS_OPT "")
    endif()

    # Remove break line
    string(REGEX REPLACE "/" "_" MODEL_SDF_TARGET_STR "RUN_MODEL_FACTORY_${ARG_CONFIG_FILENAME}")

    set(RUN_MODEL_FACTORY_FAKE "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_SDF_TARGET_STR}_model.factory.fake")

    if(EXISTS ${RUN_MODEL_FACTORY_FAKE})
        message(FATAL_ERROR "File \"${RUN_MODEL_FACTORY_FAKE}\" found, this should never be created, remove!")
    endif()    

    add_custom_target(
        ${MODEL_SDF_TARGET_STR} ALL
        DEPENDS ${RUN_MODEL_FACTORY_FAKE})

    add_custom_command(
        OUTPUT 
            ${RUN_MODEL_FACTORY_FAKE}
        COMMAND pcg-run-model-factory 
            --config-file ${ARG_CONFIG_FILENAME} 
            --store-dir ${ARG_OUTPUT_MODEL_DIR}                 
            --store-model ${OVERWRITE_MODELS_OPT}
    )

    unset(RUN_MODEL_FACTORY_FAKE)        
endfunction()
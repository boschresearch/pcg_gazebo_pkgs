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
macro(pcg_check_sdf INPUT_FILENAME)    
    if(NOT EXISTS ${INPUT_FILENAME})
        message(SEND_ERROR "Input filename has not been provided")
    else()
        message(STATUS "Checking SDF file for errors: ${INPUT_FILENAME}")
    endif()

    string(REGEX REPLACE "/" "_" MODEL_SDF_LINT_STR "MODEL_SDF_LINT_TARGET_${INPUT_FILENAME}")

    set(MODEL_SDF_LINT_FAKE "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_SDF_LINT_STR}_model.sdf.fake")

    add_custom_target(
        ${MODEL_SDF_LINT_STR} ALL
        DEPENDS ${MODEL_SDF_LINT_FAKE})

    add_custom_command(
        OUTPUT ${MODEL_SDF_LINT_FAKE}
        COMMAND rosrun pcg_gazebo sdflint --filename ${INPUT_FILENAME}
    )

    unset(MODEL_SDF_LINT_FAKE)
endmacro()
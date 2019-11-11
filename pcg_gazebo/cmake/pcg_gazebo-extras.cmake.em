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
@[if DEVELSPACE]@
set(@(PROJECT_NAME)_CMAKE_DIR @(CMAKE_CURRENT_SOURCE_DIR)/cmake)
@[else]@
set(pcg_gazebo_CMAKE_DIR ${@(PROJECT_NAME)_DIR})
@[end if]@

# Include cmake modules
file(GLOB ITEMS ${@(PROJECT_NAME)_CMAKE_DIR}/*.cmake)
foreach(ITEM ${ITEMS})
    include(${ITEM})    
endforeach()

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

"""Procedural generation package

This module implements the client for the procedural generation plugins in
Gazebo. This interface allows using Python to control the simulation state
in runtime with the help of the specific plugins write in the
**pcg_gazebo_ros_plugins**.

Example:

Attributes:
    module_level_variable1 (int):

Todo:
    * For module TODOs
"""

__all__ = ['generators', 'parsers', 'simulation']

from . import generators
from . import simulation
from . import parsers
from . import task_manager
from .path import Path
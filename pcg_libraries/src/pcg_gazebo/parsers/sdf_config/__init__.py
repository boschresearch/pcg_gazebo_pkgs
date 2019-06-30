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

import os
import sys
import inspect

from ..types import XMLBase

from .author import Author
from .description import Description
from .email import EMail
from .model import Model
from .name import Name
from .sdf import SDF
from .version import Version


def get_all_sdf_config_element_classes():
    output = list()
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase) and obj._TYPE == 'sdf_config':
                output.append(obj)
    return output


def create_sdf_config_element(tag, *args):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf_config':
                    return obj(*args)
    return None


def create_sdf_config_type(tag):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf_config':
                    return obj
    return None


def is_sdf_config_element(obj):
    return obj.__class__ in XMLBase.__subclasses__() and obj._TYPE == 'sdf_config'

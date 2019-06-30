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

from .actuator import Actuator
from .axis import Axis
from .box import Box
from .child import Child
from .collision import Collision
from .color import Color
from .cylinder import Cylinder
from .dynamics import Dynamics
from .gazebo import Gazebo
from .geometry import Geometry
from .hardware_interface import HardwareInterface
from .inertia import Inertia
from .inertial import Inertial
from .joint import Joint
from .kd import Kd
from .kp import Kp
from .limit import Limit
from .link import Link
from .mass import Mass
from .material import Material
from .max_contacts import MaxContacts
from .max_vel import MaxVel
from .mechanical_reduction import MechanicalReduction
from .mesh import Mesh
from .mimic import Mimic
from .min_depth import MinDepth
from .mu1 import Mu1
from .mu2 import Mu2
from .origin import Origin
from .parent import Parent
from .provide_feedback import ProvideFeedback
from .robot import Robot
from .safety_controller import SafetyController
from .self_collide import SelfCollide
from .sphere import Sphere
from .stopCfm import StopCFM
from .stopErp import StopERP
from .texture import Texture
from .transmission import Transmission
from .type import Type
from .visual import Visual


def get_all_urdf_element_classes():
    output = list()
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase) and obj._TYPE == 'urdf':
                output.append(obj)
    return output


def create_urdf_element(tag, *args):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'urdf':
                    return obj(*args)
    return None

def create_urdf_type(tag):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'urdf':
                    return obj
    return None

def is_urdf_element(obj):
    return obj.__class__ in types.XMLBase.__subclasses__() and obj._TYPE == 'urdf'

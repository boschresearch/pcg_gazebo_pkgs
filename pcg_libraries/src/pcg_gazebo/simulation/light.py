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
from __future__ import print_function
import collections
from ..parsers.sdf import Light as LightSDF
from .properties import Pose
from ..log import PCG_ROOT_LOGGER


class Light(object):
    def __init__(self, name='default', type='point'):
        self._sdf = LightSDF()
        self.name = name
        self.type = type
        self._pose = Pose() 

    @property
    def name(self):
        return self._sdf.name

    @name.setter
    def name(self, value):
        self._sdf.name = value

    @property
    def type(self):
        return self._sdf.type

    @type.setter
    def type(self, value):
        self._sdf.type = value

    @property
    def cast_shadows(self):
        return self._sdf.cast_shadows.value

    @cast_shadows.setter
    def cast_shadows(self, value):
        self._sdf.cast_shadows = bool(value)

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        assert isinstance(vec, collections.Iterable), \
            'Input pose vector must be iterable'
        assert len(vec) == 6 or len(vec) == 7, \
            'Pose must be given as position and Euler angles (x, y, z, ' \
            'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
            'qw, qx, qy, qz)'
        for item in vec:
            assert isinstance(item, float) or isinstance(item, int), \
                'All elements in pose vector must be a float or an integer'        
        if len(vec) == 6:
            self._pose = Pose(pos=vec[0:3], rpy=vec[3::])
        else:
            self._pose = Pose(pos=vec[0:3], quat=vec[3::])

    @property
    def diffuse(self):
        return self._sdf.diffuse.value

    @diffuse.setter
    def diffuse(self, value):
        self._sdf.diffuse = value

    @property
    def specular(self):
        return self._sdf.specular.value

    @specular.setter
    def specular(self, value):
        self._sdf.specular = value

    def set_attenuation(self, range=10, linear=1, constant=1, quadratic=0):
        self._sdf.range = range
        self._sdf.linear = linear
        self._sdf.constant = constant
        self._sdf.quadratic = quadratic

    def set_spot(self, inner_angle=0, outer_angle=0, falloff=0):
        if inner_angle > outer_angle:
            msg = 'Inner angle must be smaller than greater angle'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        if falloff < 0:
            msg = 'Falloff must be greater than or equal to zero'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        self._sdf.inner_angle = inner_angle
        self._sdf.outer_angle = outer_angle
        self._sdf.falloff = falloff

    def to_sdf(self):
        return self._sdf

    @staticmethod 
    def from_sdf(sdf):
        assert sdf._NAME == 'light', 'Only light elements can be parsed'
        light = Light()
        light._sdf = sdf
        return light

    

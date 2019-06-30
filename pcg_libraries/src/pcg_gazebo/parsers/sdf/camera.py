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

from ..types import XMLBase
from .noise import Noise
from .horizontal_fov import HorizontalFOV
from .image import Image
from .clip import Clip
from .save import Save
from .depth_camera import DepthCamera
from .distortion import Distortion


class Camera(XMLBase):
    _NAME = 'camera'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        noise=dict(creator=Noise, default=['gaussian'], optional=True),
        horizontal_fov=dict(creator=HorizontalFOV),
        image=dict(creator=Image, default=['camera']),
        clip=dict(creator=Clip), 
        save=dict(creator=Save, optional=True),
        depth_camera=dict(creator=DepthCamera, optional=True),
        distortion=dict(creator=Distortion, optional=True)
    )

    _ATTRIBUTES = dict(
        name='default'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name should be a string'
        assert len(value) > 0, 'Name should not be an empty string'
        self.attributes['name'] = value

    @property
    def noise(self):
        return self._get_child_element('noise')

    @noise.setter
    def noise(self, value):
        self._add_child_element('noise', value)

    @property
    def horizontal_fov(self):
        return self._get_child_element('horizontal_fov')

    @horizontal_fov.setter
    def horizontal_fov(self, value):
        self._add_child_element('horizontal_fov', value)

    @property
    def image(self):
        return self._get_child_element('image')

    @image.setter
    def image(self, value):
        self._add_child_element('image', value)

    @property
    def clip(self):
        return self._get_child_element('clip')

    @clip.setter
    def clip(self, value):
        self._add_child_element('clip', value)

    @property
    def save(self):
        return self._get_child_element('save')

    @save.setter
    def save(self, value):
        self._add_child_element('save', value)

    @property
    def depth_camera(self):
        return self._get_child_element('depth_camera')

    @depth_camera.setter
    def depth_camera(self, value):
        self._add_child_element('depth_camera', value)

    @property
    def distortion(self):
        return self._get_child_element('distortion')

    @distortion.setter
    def distortion(self, value):
        self._add_child_element('distortion', value)
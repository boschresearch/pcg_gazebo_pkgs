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
from .link import Link
from .joint import Joint
from .transmission import Transmission
from .gazebo import Gazebo


class Robot(XMLBase):
    _NAME = 'robot'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        link=dict(creator=Link, n_elems='+', optional=True),
        joint=dict(creator=Joint, n_elems='+', optional=True),
        gazebo=dict(creator=Gazebo, n_elems='+', optional=True),
        transmission=dict(creator=Transmission, n_elems='+', optional=True)
    )

    _ATTRIBUTES = dict(
        name='robot'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self.attributes['name'] = value

    @property
    def links(self):
        return self._get_child_element('link')

    @property
    def joints(self):
        return self._get_child_element('joint')

    @property
    def gazebos(self):
        return self._get_child_element('gazebo')

    @property
    def transmissions(self):
        return self._get_child_element('transmission')

    def add_link(self, name, link=None):
        if self.links is not None:
            for elem in self.links:
                if elem.name == name:
                    print('Link element with name {} already exists'.format(name))
                    return
        if link is not None:
            self._add_child_element('link', link)
        else:
            link = Link()
            self._add_child_element('link', link)
        self.children['link'][-1].name = name

    def get_link_by_name(self, name):
        if self.links is None:
            return None
        else:
            for elem in self.links:
                if elem.name == name:
                    return elem
        return None

    def add_joint(self, name, joint=None):
        if self.joints is not None:
            for elem in self.joints:
                if elem.name == name:
                    print('Joint element with name {} already exists'.format(name))
                    return
        if joint is not None:
            self._add_child_element('joint', joint)
        else:
            joint = Joint()
            self._add_child_element('joint', joint)
        self.children['joint'][-1].name = name

    def get_joint_by_name(self, name):
        if self.joints is None:
            return None
        else:
            for elem in self.joints:
                if elem.name == name:
                    return elem
        return None

    def add_gazebo(self, reference=None, gazebo=None):
        if gazebo is not None:
            self._add_child_element('gazebo', gazebo)
        else:
            gazebo = Gazebo()
            self._add_child_element('gazebo', gazebo)
        if reference != None:
            self.children['gazebo'][-1].reference = reference

    def add_transmission(self, name, transmission=None):
        if self.transmissions is not None:
            for elem in self.transmissions:
                if elem.name == name:
                    print('Transmission element with name {} already exists'.format(name))
                    return
        if transmission is not None:
            self._add_child_element('transmission', transmission)
        else:
            transmission = Transmission()
            self._add_child_element('transmission', transmission)
        self.children['transmission'][-1].name = name

    def get_transmission_by_name(self, name):
        if self.transmissions is None:
            return None
        else:
            for elem in self.transmissions:
                if elem.name == name:
                    return elem
        return None

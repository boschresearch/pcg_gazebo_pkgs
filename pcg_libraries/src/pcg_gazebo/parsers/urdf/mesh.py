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


class Mesh(XMLBase):
    _NAME = 'mesh'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        filename=''
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def filename(self):
        return self.attributes['filename']

    @filename.setter
    def filename(self, value):
        assert isinstance(value, str) or isinstance(value, unicode), \
            'Filename must be a string or unicode'
        self.attributes['filename'] = value

    def to_sdf(self):
        from ..sdf import create_sdf_element
        
        obj = create_sdf_element('mesh')
        obj.uri = self.filename
        return obj

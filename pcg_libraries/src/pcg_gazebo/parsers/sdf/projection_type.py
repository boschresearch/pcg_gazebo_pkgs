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
from ..types import XMLString


class ProjectionType(XMLString):
    _NAME = 'projection_type'
    _TYPE = 'sdf'

    _VALUE_OPTIONS = ['perspective', 'orthographic']

    def __init__(self, default='perspective'):
        XMLString.__init__(self, default)

    def _set_value(self, value):
        assert value in self._VALUE_OPTIONS, \
            '[{}] options are {}'.format(self.xml_element_name, 
            self._VALUE_OPTIONS)
        self._value = value
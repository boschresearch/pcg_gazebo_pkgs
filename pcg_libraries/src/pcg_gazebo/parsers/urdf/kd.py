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

from ..types import XMLScalar


class Kd(XMLScalar):
    _NAME = 'kd'
    _TYPE = 'urdf'

    def __init__(self, default=1):
        XMLScalar.__init__(self, default)

    def _set_value(self, value):
        assert value >= 0, 'Input value must be equal or greater than zero'
        XMLScalar._set_value(self, value)

    def to_sdf(self):
        from ..sdf import create_sdf_element
        
        obj = create_sdf_element('kd')
        obj.value = self.value
        return obj

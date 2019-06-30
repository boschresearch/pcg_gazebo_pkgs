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
from ..types import XMLVector


class Specular(XMLVector):
    _NAME = 'specular'
    _TYPE = 'sdf'

    def __init__(self, default=[0.1, 0.1, 0.1, 1]):
        XMLVector.__init__(self, 4)
        self._default = default
        self._value = default

    def _set_value(self, value):
        assert self._is_numeric_vector(value, [0, 1]), \
            'Input is not a valid specular vector'        
        value = list(value)        
        XMLVector._set_value(self, value)

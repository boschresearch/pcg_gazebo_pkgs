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

from . import XMLBase
import collections


class XMLVector(XMLBase):
    _NAME = ''

    def __init__(self, size=None):
        XMLBase.__init__(self)
        assert size is not None, 'Vector size cannot be None'
        assert isinstance(size, int), \
            '[{}] Vector size input must be an integer, received={}'.format(
                self.xml_element_name, size)
        assert size > 0, '[{}] Size must be greater than zero'.format(
            self.xml_element_name)
        self._size = size
        self._value = [0 for _ in range(self._size)]

    def _set_value(self, value):
        assert isinstance(value, collections.Iterable), \
            'Input must be iterable, element={}, received={}, type={}'.format(
                self._NAME, value, type(value))
        assert len(list(value)) == self._size, \
            'Input vector has the wrong size, element={}, received={}, ' \
            'size of received={}, expected length={}'.format(
                self._NAME, value, len(list(value)), self._size)
        for item in value:
            assert isinstance(item, float) or isinstance(item, int)
        self._value = list(value)

    def reset(self):
        self._value = [0 for _ in range(self._size)]
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, list):
            print('Vector object must have a list as value')
            return False
        if len(self._value) != self._size:
            print('Normal value must be a list with 3 elements')
            return False
        for item in self._value:
            if not isinstance(item, float) and not isinstance(item, int):
                print('Each vector element must be a float or integer')
                return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid vector'
        output_str = ' '.join(['{}'] * self._size)
        return output_str.format(*[format(x, 'n') for x in self._value])

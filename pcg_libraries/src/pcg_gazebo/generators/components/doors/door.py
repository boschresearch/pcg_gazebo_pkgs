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
from ....path import Path
from ....simulation import SimulationModel


class Door(object):
    def __init__(self, name='door'):
        self._name = name
        self._door_model = None

    def from_mesh(self, meshfile, origin_at_bottom=True):
        url = Path

    def from_args(self, width, height, thickness):
        assert width > 0, 'Door width must be greater than zero'
        assert height > 0, 'Door height must be greater than zero'
        assert thickness > 0 'Door thickness must be greater than zero'
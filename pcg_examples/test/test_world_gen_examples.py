#!/usr/bin/env python
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
import roslib
import unittest
import rospkg
from pcg_gazebo.generators import WorldGenerator

PKG = 'pcg_examples'
roslib.load_manifest(PKG)

CWD = rospkg.RosPack().get_path(PKG)


class TestWorldGenExamples(unittest.TestCase):
    def test_world_gen_from_yaml(self):
        root_path = os.path.join(CWD, 'config', 'world_generator', 'worlds')

        for config_file in os.listdir(root_path):
            if not os.path.isfile(os.path.join(root_path, config_file)):
                continue
            gen = WorldGenerator()
            gen.from_yaml(os.path.join(root_path, config_file))
            gen.run_engines()
            self.assertGreater(len(gen.world.models), 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_world_gen_examples', TestWorldGenExamples)
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
import roslib
import unittest
import rospkg
import os
from pcg_gazebo.utils import load_yaml
from pcg_gazebo.simulation import SimulationModel
from pcg_gazebo.generators.creators import create_models_from_config

PKG = 'pcg_examples'
roslib.load_manifest(PKG)

CWD = rospkg.RosPack().get_path(PKG)


class TestModelFactoryExamples(unittest.TestCase):
    def test_model_generations(self):
        root_path = os.path.join(CWD, 'config', 'model_factory')

        for model_factory_config_file in os.listdir(root_path):
            if not os.path.isfile(os.path.join(root_path, model_factory_config_file)):
                continue
            config = load_yaml(os.path.join(root_path, model_factory_config_file))
            models = create_models_from_config(config)
            self.assertGreater(len(models), 0)
            for model in models:
                self.assertIsInstance(model, SimulationModel)
            
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_model_factory_examples', TestModelFactoryExamples)
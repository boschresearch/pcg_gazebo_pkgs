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
from pcg_gazebo.utils import process_jinja_template
from pcg_gazebo.parsers import parse_sdf

PKG = 'pcg_examples'
roslib.load_manifest(PKG)

CWD = rospkg.RosPack().get_path(PKG)


class TestTemplateExamples(unittest.TestCase):
    def test_template_models(self):
        root_path = os.path.join(CWD, 'templates', 'models')

        for template_file in os.listdir(root_path):
            if not os.path.isfile(os.path.join(root_path, template_file)):
                continue
            output_xml = process_jinja_template(os.path.join(root_path, template_file))
            sdf = parse_sdf(output_xml)
            self.assertIsNotNone(sdf)
            self.assertEqual(sdf.xml_element_name, 'sdf')
            self.assertIsNotNone(sdf.models)
            self.assertEqual(len(sdf.models), 1)

    def test_template_world_gen(self):
        root_path = os.path.join(CWD, 'templates', 'world_generator')

        for template_file in os.listdir(root_path):
            if not os.path.isfile(os.path.join(root_path, template_file)):
                continue
            output_xml = process_jinja_template(os.path.join(root_path, template_file))
            sdf = parse_sdf(output_xml)
            self.assertIsNotNone(sdf)
            self.assertEqual(sdf.xml_element_name, 'sdf')
            self.assertIsNotNone(sdf.world)
            
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_template_examples', TestTemplateExamples)
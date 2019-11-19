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
import random
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.parsers import parse_sdf
from pcg_gazebo.parsers.sdf import create_sdf_element
from pcg_gazebo.parsers.types import XMLScalar, XMLVector, XMLString, \
    XMLInteger, XMLBoolean

PKG = 'pcg_libraries'
roslib.load_manifest(PKG)


class TestParseSDF(unittest.TestCase):
    def test_parse_string_sdf_strings(self):
        def generate_string_test_obj(name, value=None):            
            if value is None:
                value = generate_random_string(5)
            sdf_str = '<{}>{}</{}>'.format(name, value, name)
            expected_sdf = create_sdf_element(name)
            self.assertIsNotNone(expected_sdf, '{} returned None'.format(name))
            expected_sdf.value = value
            return sdf_str, expected_sdf

        string_test_cases = list()

        for c in XMLString.__subclasses__():
            if c._TYPE == 'sdf':
                string_test_cases.append(c._NAME)
        
        # Exclude special cases
        exclude = ['empty', 'format', 'friction_model', 'collision', 
            'measure_direction', 'localization', 'view_controller']

        for tag in string_test_cases:
            if tag in exclude:
                continue            
            sdf_str, expected_sdf = generate_string_test_obj(tag) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        # Add tests for measure_direction
        values = ['child_to_parent', 'parent_to_link']
        for value in values:
            sdf_str, expected_sdf = generate_string_test_obj('measure_direction', value) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        # Add tests for format
        values = ['L8', 'R8G8B8', 'B8G8R8', 'BAYER_RGGB8',
            'BAYER_BGGR8', 'BAYER_GBRG8', 'BAYER_GRBG8']
        for value in values:
            sdf_str, expected_sdf = generate_string_test_obj('format', value) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)
        
        # Add tests for friction_model
        values = ['pyramid_model', 'box_model', 'cone_model']
        for value in values:
            sdf_str, expected_sdf = generate_string_test_obj('friction_model', value) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        # Add tests for localization
        values = ['CUSTOM', 'NED', 'ENU', 'NWU', 'GRAV_UP', 'GRAV_DOWN']
        for value in values:
            sdf_str, expected_sdf = generate_string_test_obj('localization', value) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

    def test_parse_boolean_sdf_strings(self):
        def generate_boolean_test_obj(name, value=None):            
            if value is None:
                value = random.choice([True, False])
            sdf_str = '<{}>{}</{}>'.format(name, int(value), name)
            expected_sdf = create_sdf_element(name)
            self.assertIsNotNone(expected_sdf, '{} returned None'.format(name))
            expected_sdf.value = value
            return sdf_str, expected_sdf

        int_test_cases = list()

        for c in XMLBoolean.__subclasses__():
            if c._TYPE == 'sdf':
                int_test_cases.append(c._NAME)
        
        # Exclude special cases
        exclude = []

        for tag in int_test_cases:
            if tag in exclude:
                continue            
            sdf_str, expected_sdf = generate_boolean_test_obj(tag) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

    def test_parse_integer_sdf_strings(self):
        def generate_integer_test_obj(name, value=None):            
            if value is None:
                value = random.randint(1, 10)
            sdf_str = '<{}>{}</{}>'.format(name, value, name)
            expected_sdf = create_sdf_element(name)
            self.assertIsNotNone(expected_sdf, '{} returned None'.format(name))
            expected_sdf.value = value
            return sdf_str, expected_sdf

        int_test_cases = list()

        for c in XMLInteger.__subclasses__():
            if c._TYPE == 'sdf':
                int_test_cases.append(c._NAME)
        
        # Exclude special cases
        exclude = []

        for tag in int_test_cases:
            if tag in exclude:
                continue            
            sdf_str, expected_sdf = generate_integer_test_obj(tag) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

    def test_parse_scalar_sdf_strings(self):
        def generate_scalar_test_obj(name, value=None):            
            if value is None:
                value = random.random()
            sdf_str = '<{}>{}</{}>'.format(name, value, name)
            expected_sdf = create_sdf_element(name)
            self.assertIsNotNone(expected_sdf, '{} returned None'.format(name))
            expected_sdf.value = value
            return sdf_str, expected_sdf

        scalar_test_cases = list()

        for c in XMLScalar.__subclasses__():
            if c._TYPE == 'sdf':
                scalar_test_cases.append(c._NAME)
        
        # Exclude special cases
        exclude = ['friction', 'range', 'poissons_ratio']

        for tag in scalar_test_cases:
            if tag in exclude:
                continue            
            sdf_str, expected_sdf = generate_scalar_test_obj(tag) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        #TODO Add tests for friction and range
        sdf_str, expected_sdf = generate_scalar_test_obj('poissons_ratio', 
            random.uniform(-1, 0.5)) 
        sdf = parse_sdf(sdf_str)
        self.assertIsNotNone(sdf)
        self.assertEqual(parse_sdf(sdf_str), expected_sdf)

    def test_parse_vector_sdf_strings(self):
        def generate_array_test_obj(name, vec=None):
            expected_sdf = create_sdf_element(name)            
            self.assertIsNotNone(expected_sdf, '{} returned None'.format(name))
            if vec is None:
                vec = [random.random() for _ in range(expected_sdf._size)]
            sdf_str = '<{}>{}</{}>'.format(name, ' '.join(str(x) for x in vec), name)
            expected_sdf.value = vec
            return sdf_str, expected_sdf

        array_test_cases = list()

        for c in XMLVector.__subclasses__():
            if c._TYPE == 'sdf':
                array_test_cases.append(c._NAME)

        exclude = ['xyz']

        for tag in array_test_cases:            
            if tag in exclude:
                continue
            sdf_str, expected_sdf = generate_array_test_obj(tag) 
            sdf = parse_sdf(sdf_str)
            self.assertIsNotNone(sdf)
            self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        # Test for XYZ vectors
        sdf_str, expected_sdf = generate_array_test_obj('xyz', [1, 0, 0]) 
        sdf = parse_sdf(sdf_str)
        self.assertIsNotNone(sdf)
        self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        sdf_str, expected_sdf = generate_array_test_obj('xyz', [0, 1, 0]) 
        sdf = parse_sdf(sdf_str)
        self.assertIsNotNone(sdf)
        self.assertEqual(parse_sdf(sdf_str), expected_sdf)

        sdf_str, expected_sdf = generate_array_test_obj('xyz', [0, 0, 1]) 
        sdf = parse_sdf(sdf_str)
        self.assertIsNotNone(sdf)
        self.assertEqual(parse_sdf(sdf_str), expected_sdf)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_module', TestParseSDF)
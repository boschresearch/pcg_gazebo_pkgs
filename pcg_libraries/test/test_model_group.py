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
import numpy
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.simulation import ModelGroup
from pcg_gazebo.simulation.properties import Pose
from pcg_gazebo.generators.creators import box_factory

PKG = 'pcg_libraries'
roslib.load_manifest(PKG)


class TestModelGroup(unittest.TestCase):
    def test_group_name(self):
        valid_name = generate_random_string(size=3)
        group = ModelGroup(name=valid_name)
        self.assertEqual(valid_name, group.name)

        invalid_names = [
            10,
            None,
            dict(),
            list(),
            ''
        ]

        for name in invalid_names:
            with self.assertRaises(AssertionError):
                group = ModelGroup(name=name)        

    def test_group_init_pose(self):
        # Set initial pose with Pose object
        pose = Pose.random()
        group = ModelGroup(pose=pose)
        
        self.assertEqual(numpy.sum(group.pose.position - pose.position), 0)
        self.assertEqual(numpy.sum(group.pose.quat - pose.quat), 0)

        # Test invalid pose objects
        invalid_poses = [
            dict(),
            list(),
            'asd',
            '',
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            None
        ]

        for pose in invalid_poses:
            with self.assertRaises(AssertionError):
                group.pose = pose

    def test_add_models(self):
        group = ModelGroup()
        boxes = box_factory(
            size="__import__('numpy').random.random((2, 3))",
            mass="__import__('numpy').random.random(2)",
            use_permutation=True,
            name='box'
        )

        self.assertEqual(len(boxes), 4)

        for box in boxes:
            group.add_model(box.name, box)
            self.assertIn(box.name, group.models)
            self.assertTrue(group.model_exists(box.name))

        self.assertEqual(group.n_models, 4)

        meshes = group.get_meshes()
        self.assertEqual(len(meshes), 4)
        
        group.reset_models()
        self.assertEqual(group.n_models, 0)

        for box in boxes:
            group.add_model(box.name, box)
        self.assertEqual(group.n_models, 4)

        for box in boxes:
            self.assertTrue(group.rm_model(box.name))
        self.assertEqual(group.n_models, 0)

    def test_bounds(self):
        box_1 = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box_1'
        )[0]

        self.assertTrue(box_1.name, 'box_1')

        box_2 = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box_2'
        )[0]       

        self.assertTrue(box_2.name, 'box_2')

        # Test bounds for different positions of the boxes
        box_1.pose = Pose.random_position()
        box_2.pose = Pose.random_position()

        group = ModelGroup()
        group.add_model(box_1.name, box_1)
        group.add_model(box_2.name, box_2)

        self.assertEqual(group.n_models, 2)

        bounds = group.get_bounds()

        ref_bounds = numpy.array([
            [numpy.min([box_1.pose.x, box_2.pose.x]) - 1, numpy.min([box_1.pose.y, box_2.pose.y]) - 1, numpy.min([box_1.pose.z, box_2.pose.z]) - 1],
            [numpy.max([box_1.pose.x, box_2.pose.x]) + 1, numpy.max([box_1.pose.y, box_2.pose.y]) + 1, numpy.max([box_1.pose.z, box_2.pose.z]) + 1]
        ])

        self.assertAlmostEqual(numpy.sum(ref_bounds - bounds), 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_model_group', TestModelGroup)
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
import string
import numpy as np
from pcg_gazebo.parsers.sdf_config import create_sdf_config_element
from pcg_gazebo.generators.creators import create_models_from_config
from pcg_gazebo.task_manager import GazeboProxy
from pcg_gazebo.simulation.properties import Material, Pose
from pcg_gazebo.simulation import SimulationModel

PKG = 'pcg_libraries'
roslib.load_manifest(PKG)


def _get_random_string(size=3):
    return ''.join(random.choice(string.ascii_letters) for i in range(size))

def _get_colors():
    return [None, 'xkcd', 'random'] + \
        [ random.choice(list(Material.get_xkcd_colors_list().keys())) for _ in range(2)] + \
            [ [random.random() for _ in range(4)] for _ in range(2) ]

class TestModelFactory(unittest.TestCase):
    def test_static_box_model(self):        
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            size = [random.random() for _ in range(3)]
            model_config = [
                dict(
                    type='box',
                    args=dict(
                        size=size,
                        name=name,
                        pose=pose,
                        color=color,
                        collision_parameters=dict(
                            mu=random.uniform(0, 10),
                            mu2=random.uniform(0, 10),
                            friction=random.uniform(0, 10),
                            friction2=random.uniform(0, 10),
                            slip1=random.uniform(0, 1),
                            slip2=random.uniform(0, 1),
                            rolling_friction=random.uniform(0, 1),
                            fdir1=[0, 0, 0],
                            max_contacts=1,
                            soft_cfm=random.uniform(0, 10),
                            soft_erp=random.uniform(0, 10),
                            kp=random.uniform(0, 100000),
                            kd=random.uniform(0, 10),
                            max_vel=random.uniform(0, 0.1),
                            min_depth=random.uniform(0, 0.1),
                            split_impulse=random.choice([True, False]),
                            split_impulse_penetration_threshold=-0.01,
                            enable_friction=True,
                            enable_contact=True
                        )
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertTrue(models[0].static)
            self.assertEqual(len(models[0].links), 1)

            link_name = models[0].link_names[0]
            # Test visual element
            self.assertEqual(len(models[0].links[link_name].visuals), 1)
            geometry = models[0].links[link_name].visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'box')
            self.assertEqual(geometry.get_param('size'), size)

            # Test collision element
            self.assertEqual(len(models[0].links[link_name].collisions), 1)
            collision = models[0].links[link_name].get_collision_by_name('collision')

            tags = ['mu', 'mu2', 'slip1', 'slip2', 'fdir1']
            for tag in tags:
                self.assertEqual(
                    collision.get_ode_friction_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])
            
            tags = ['friction', 'friction2', 'rolling_friction', 'fdir1']
            for tag in tags:
                self.assertEqual(
                    collision.get_bullet_friction_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])

            tags = ['soft_cfm', 'soft_erp', 'kp', 'kd', 'max_vel', 'min_depth']
            for tag in tags:
                self.assertEqual(
                    collision.get_ode_contact_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])

            tags = ['soft_cfm', 'soft_erp', 'kp', 'kd', 'split_impulse', \
                'split_impulse_penetration_threshold']
            for tag in tags:
                self.assertEqual(
                    collision.get_bullet_contact_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])
            
            geometry = collision.geometry
            self.assertEqual(geometry.get_type(), 'box')
            self.assertEqual(geometry.get_param('size'), size)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)            

    def test_dynamic_box_model(self):
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            size = [random.random() for _ in range(3)]
            mass = random.random()
            model_config = [
                dict(
                    type='box',
                    args=dict(
                        size=size,
                        name=name,
                        pose=pose,
                        mass=mass,
                        color=color,
                        collision_parameters=dict(
                            mu=random.uniform(0, 10),
                            mu2=random.uniform(0, 10),
                            friction=random.uniform(0, 10),
                            friction2=random.uniform(0, 10),
                            slip1=random.uniform(0, 1),
                            slip2=random.uniform(0, 1),
                            rolling_friction=random.uniform(0, 1),
                            fdir1=[0, 0, 0],
                            max_contacts=1,
                            soft_cfm=random.uniform(0, 10),
                            soft_erp=random.uniform(0, 10),
                            kp=random.uniform(0, 100000),
                            kd=random.uniform(0, 10),
                            max_vel=random.uniform(0, 0.1),
                            min_depth=random.uniform(0, 0.1),
                            split_impulse=random.choice([True, False]),
                            split_impulse_penetration_threshold=-0.01,
                            enable_friction=True,
                            enable_contact=True
                        )
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertFalse(models[0].static)
            self.assertEqual(len(models[0].links), 1)
            

            link_name = models[0].link_names[0]
            # Test link properties
            link = models[0].links[link_name]
            self.assertEqual(link.inertial.mass, mass)
            self.assertAlmostEqual(link.inertial.ixx, 1. / 12 * mass * (size[1]**2 + size[2]**2))
            self.assertAlmostEqual(link.inertial.iyy, 1. / 12 * mass * (size[0]**2 + size[2]**2))
            self.assertAlmostEqual(link.inertial.izz, 1. / 12 * mass * (size[0]**2 + size[1]**2))
            self.assertEqual(link.inertial.ixy, 0)
            self.assertEqual(link.inertial.ixz, 0)
            self.assertEqual(link.inertial.iyz, 0)

            # Test visual element
            self.assertEqual(len(link.visuals), 1)
            geometry = link.visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'box')
            self.assertEqual(geometry.get_param('size'), size)

            # Test collision element
            self.assertEqual(len(link.collisions), 1)
            collision = link.get_collision_by_name('collision')

            tags = ['mu', 'mu2', 'slip1', 'slip2', 'fdir1']
            for tag in tags:
                self.assertEqual(
                    collision.get_ode_friction_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])
            
            tags = ['friction', 'friction2', 'rolling_friction', 'fdir1']
            for tag in tags:
                self.assertEqual(
                    collision.get_bullet_friction_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])

            tags = ['soft_cfm', 'soft_erp', 'kp', 'kd', 'max_vel', 'min_depth']
            for tag in tags:
                self.assertEqual(
                    collision.get_ode_contact_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])

            tags = ['soft_cfm', 'soft_erp', 'kp', 'kd', 'split_impulse', \
                'split_impulse_penetration_threshold']
            for tag in tags:
                self.assertEqual(
                    collision.get_bullet_contact_param(tag),
                    model_config[0]['args']['collision_parameters'][tag])
            
            geometry = collision.geometry
            self.assertEqual(geometry.get_type(), 'box')
            self.assertEqual(geometry.get_param('size'), size)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)
        
    def test_box_factory_fixed_args_with_permutation(self):
        for color in _get_colors():
            n_sizes = random.randint(2, 5)
            n_masses = random.randint(2, 5)
            name = _get_random_string(3)
            sizes = [ [random.random() for _ in range(3)] for _ in range(n_sizes) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_masses)]
            model_config = [
                dict(
                    type='box_factory',
                    args=dict(
                        name=name,
                        size=sizes,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_sizes * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                self.assertIn(geometry.get_param('size'), sizes)
                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                self.assertIn(geometry.get_param('size'), sizes)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_box_factory_fixed_args_no_permutation(self):
        for color in _get_colors():
            n_models = random.randint(2, 5)            
            name = _get_random_string(3)
            sizes = [ [random.random() for _ in range(3)] for _ in range(n_models) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_models)]
            model_config = [
                dict(
                    type='box_factory',
                    args=dict(
                        name=name,
                        size=sizes,
                        mass=masses,
                        pose=pose,
                        use_permutation=False,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_models)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                self.assertIn(geometry.get_param('size'), sizes)
                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                self.assertIn(geometry.get_param('size'), sizes)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_box_factory_lambda_args_with_permutation(self):
        for color in _get_colors():
            n_sizes = random.randint(2, 5)
            n_masses = random.randint(2, 5)
            name = _get_random_string(3)
            sizes = "__import__('numpy').random.random(({}, 3))".format(n_sizes)
            masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)
            pose = [random.random() for _ in range(6)]
            model_config = [
                dict(
                    type='box_factory',
                    args=dict(
                        name=name,
                        size=sizes,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_sizes * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'box')
                                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def text_box_forced_permutation(self):
        n_sizes = 2
        n_masses = 3
        name = _get_random_string(3)

        # Test with lambda functions
        sizes = "__import__('numpy').random.random(({}, 3))".format(n_sizes)
        masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)
        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='box_factory',
                args=dict(
                    name=name,
                    size=sizes,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)

        self.assertEqual(len(models), n_sizes * n_masses)

        # Test with fixed arguments
        sizes = [ [random.random() for _ in range(3)] for _ in range(n_sizes) ]
        masses = [ random.random() for _ in range(n_masses) ]
        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='box_factory',
                args=dict(
                    name=name,
                    size=sizes,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)

        self.assertEqual(len(models), n_sizes * n_masses)

    def test_static_cylinder_model(self):        
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            radius = random.random()
            length = random.random()
            model_config = [
                dict(
                    type='cylinder',
                    args=dict(
                        radius=radius,
                        length=length,
                        name=name,
                        pose=pose,
                        color=color
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertTrue(models[0].static)
            self.assertEqual(len(models[0].links), 1)

            link_name = models[0].link_names[0]
            # Test visual element
            self.assertEqual(len(models[0].links[link_name].visuals), 1)
            geometry = models[0].links[link_name].visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'cylinder')
            self.assertEqual(geometry.get_param('radius'), radius)
            self.assertEqual(geometry.get_param('length'), length)

            # Test collision element
            self.assertEqual(len(models[0].links[link_name].collisions), 1)
            geometry = models[0].links[link_name].collisions[0].geometry
            self.assertEqual(geometry.get_type(), 'cylinder')
            self.assertEqual(geometry.get_param('radius'), radius)
            self.assertEqual(geometry.get_param('length'), length)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)

    def test_dynamic_cylinder_model(self):        
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            radius = random.random()
            length = random.random()
            mass = random.random()
            model_config = [
                dict(
                    type='cylinder',
                    args=dict(
                        radius=radius,
                        length=length,
                        mass=mass,
                        name=name,
                        pose=pose,
                        color=color
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertFalse(models[0].static)
            self.assertEqual(len(models[0].links), 1)

            link_name = models[0].link_names[0]

            # Test link properties
            link = models[0].links[link_name]
            self.assertEqual(link.inertial.mass, mass)
            self.assertAlmostEqual(link.inertial.ixx, 1. / 12 * mass * (3 * radius**2 + length**2))
            self.assertAlmostEqual(link.inertial.iyy, 1. / 12 * mass * (3 * radius**2 + length**2))
            self.assertAlmostEqual(link.inertial.izz, 0.5 * mass * radius**2)
            self.assertEqual(link.inertial.ixy, 0)
            self.assertEqual(link.inertial.ixz, 0)
            self.assertEqual(link.inertial.iyz, 0)

            # Test visual element
            self.assertEqual(len(models[0].links[link_name].visuals), 1)
            geometry = models[0].links[link_name].visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'cylinder')
            self.assertEqual(geometry.get_param('radius'), radius)
            self.assertEqual(geometry.get_param('length'), length)

            # Test collision element
            self.assertEqual(len(models[0].links[link_name].collisions), 1)
            geometry = models[0].links[link_name].collisions[0].geometry
            self.assertEqual(geometry.get_type(), 'cylinder')
            self.assertEqual(geometry.get_param('radius'), radius)
            self.assertEqual(geometry.get_param('length'), length)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)

    def test_cylinder_factory_fixed_args_with_permutation(self):
        for color in _get_colors():
            n_radius = random.randint(2, 4)
            n_length = random.randint(2, 4)
            n_masses = random.randint(2, 4)
            name = _get_random_string(3)
            radius = [ random.random() for _ in range(n_radius) ]
            length = [ random.random() for _ in range(n_length) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_masses)]
            model_config = [
                dict(
                    type='cylinder_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        length=length,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_radius * n_length * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                self.assertIn(geometry.get_param('radius'), radius)
                self.assertIn(geometry.get_param('length'), length)
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                self.assertIn(geometry.get_param('radius'), radius)
                self.assertIn(geometry.get_param('length'), length)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)
        
    def test_cylinder_factory_fixed_args_no_permutation(self):
        for color in _get_colors():
            n_models = random.randint(2, 4)
            name = _get_random_string(3)
            radius = [ random.random() for _ in range(n_models) ]
            length = [ random.random() for _ in range(n_models) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_models)]
            model_config = [
                dict(
                    type='cylinder_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        length=length,
                        mass=masses,
                        pose=pose,
                        use_permutation=False,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_models)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                self.assertIn(geometry.get_param('radius'), radius)
                self.assertIn(geometry.get_param('length'), length)
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                self.assertIn(geometry.get_param('radius'), radius)
                self.assertIn(geometry.get_param('length'), length)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_cylinder_factory_lambda_args_with_permutation(self):
        for color in _get_colors():
            n_radius = random.randint(2, 4)
            n_length = random.randint(2, 4)
            n_masses = random.randint(2, 4)
            name = _get_random_string(3)

            radius = "__import__('numpy').random.random({})".format(n_radius)
            length = "__import__('numpy').linspace(1, 10, {})".format(n_length)
            masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)

            pose = [random.random() for _ in range(6)]
            model_config = [
                dict(
                    type='cylinder_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        length=length,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_radius * n_length * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'cylinder')
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_cylinder_forced_permutation(self):
        n_radius = 1
        n_masses = 2
        n_length = 3
        name = _get_random_string(3)

        # Test with lambda functions        
        radius = "__import__('numpy').random.random({})".format(n_radius)
        length = "__import__('numpy').random.random({})".format(n_length)
        masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)

        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='cylinder_factory',
                args=dict(
                    name=name,
                    radius=radius,
                    length=length,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)
        self.assertEqual(len(models), n_radius * n_length * n_masses)

        # Test with fixed arguments
        radius = [random.random() for _ in range(n_radius)]
        length = [random.random() for _ in range(n_length)]
        masses = [random.random() for _ in range(n_masses)]

        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='cylinder_factory',
                args=dict(
                    name=name,
                    radius=radius,
                    length=length,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)
        self.assertEqual(len(models), n_radius * n_length * n_masses)

    def test_static_sphere_model(self):        
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            radius = random.random()
            model_config = [
                dict(
                    type='sphere',
                    args=dict(
                        radius=radius,
                        name=name,
                        pose=pose,
                        color=color
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertTrue(models[0].static)
            self.assertEqual(len(models[0].links), 1)

            link_name = models[0].link_names[0]
            # Test visual element
            self.assertEqual(len(models[0].links[link_name].visuals), 1)
            geometry = models[0].links[link_name].visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'sphere')
            self.assertEqual(geometry.get_param('radius'), radius)

            # Test collision element
            self.assertEqual(len(models[0].links[link_name].collisions), 1)
            geometry = models[0].links[link_name].collisions[0].geometry
            self.assertEqual(geometry.get_type(), 'sphere')
            self.assertEqual(geometry.get_param('radius'), radius)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)

    def test_dynamic_sphere_model(self):        
        for color in _get_colors():
            name = _get_random_string(3)
            pose = [random.random() for _ in range(6)]
            radius = random.random()
            mass = random.random()
            model_config = [
                dict(
                    type='sphere',
                    args=dict(
                        radius=radius,
                        mass=mass,
                        name=name,
                        pose=pose,
                        color=color
                    ))
                ]
            models = create_models_from_config(model_config)

            self.assertEqual(len(models), 1)
            self.assertIsInstance(models[0], SimulationModel)
            # Test pose of the model
            self.assertEqual(models[0].pose.position.tolist(), pose[0:3])
            q = Pose.rpy2quat(*pose[3::])
            diff = Pose.get_transform(models[0].pose.quat, q)

            # Test model properties
            self.assertAlmostEqual(np.sum(diff[0:3]), 0)
            self.assertFalse(models[0].static)
            self.assertEqual(len(models[0].links), 1)

            link_name = models[0].link_names[0]
            # Test link properties
            link = models[0].links[link_name]
            self.assertEqual(link.inertial.mass, mass)
            inertia = 2. / 5 * mass * radius**2
            self.assertAlmostEqual(link.inertial.ixx, inertia)
            self.assertAlmostEqual(link.inertial.iyy, inertia)
            self.assertAlmostEqual(link.inertial.izz, inertia)
            self.assertEqual(link.inertial.ixy, 0)
            self.assertEqual(link.inertial.ixz, 0)
            self.assertEqual(link.inertial.iyz, 0)

            # Test visual element
            self.assertEqual(len(models[0].links[link_name].visuals), 1)
            geometry = models[0].links[link_name].visuals[0].geometry
            self.assertEqual(geometry.get_type(), 'sphere')
            self.assertEqual(geometry.get_param('radius'), radius)

            # Test collision element
            self.assertEqual(len(models[0].links[link_name].collisions), 1)
            geometry = models[0].links[link_name].collisions[0].geometry
            self.assertEqual(geometry.get_type(), 'sphere')
            self.assertEqual(geometry.get_param('radius'), radius)

            # Test color exists
            material = models[0].links[link_name].visuals[0].material
            self.assertIsNotNone(material)
            self.assertIsNotNone(material.ambient)
            self.assertIsNotNone(material.diffuse)

            if not isinstance(color, str) and isinstance(color, list) and color is not None:
                self.assertEqual(material.ambient.value, color)
                self.assertEqual(material.diffuse.value, color)

    def test_sphere_factory_fixed_args_with_permutation(self):
        for color in _get_colors():
            n_radius = random.randint(2, 4)
            n_masses = random.randint(2, 4)
            name = _get_random_string(3)
            radius = [ random.random() for _ in range(n_radius) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_masses)]
            model_config = [
                dict(
                    type='sphere_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_radius * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                self.assertIn(geometry.get_param('radius'), radius)
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                self.assertIn(geometry.get_param('radius'), radius)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_sphere_factory_fixed_args_no_permutation(self):
        for color in _get_colors():
            n_models = random.randint(2, 4)
            name = _get_random_string(3)
            radius = [ random.random() for _ in range(n_models) ]
            pose = [random.random() for _ in range(6)]
            masses = [random.random() for _ in range(n_models)]
            model_config = [
                dict(
                    type='sphere_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        mass=masses,
                        pose=pose,
                        use_permutation=False,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_models)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                self.assertIn(models[i].links[link_name].inertial.mass, masses)
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                self.assertIn(geometry.get_param('radius'), radius)
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                self.assertIn(geometry.get_param('radius'), radius)
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_sphere_factory_lambda_args_with_permutation(self):
        for color in _get_colors():
            n_radius = random.randint(2, 4)
            n_masses = random.randint(2, 4)
            name = _get_random_string(3)

            radius = "__import__('numpy').random.random({})".format(n_radius)
            masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)

            pose = [random.random() for _ in range(6)]
            model_config = [
                dict(
                    type='sphere_factory',
                    args=dict(
                        name=name,
                        radius=radius,
                        mass=masses,
                        pose=pose,
                        use_permutation=True,
                        color=color
                    ))
            ]

            models = create_models_from_config(model_config)

            self.assertEqual(len(models), n_radius * n_masses)

            for i in range(len(models)):            
                # Check the name generator with counter
                self.assertIn(name + '_', models[i].name)
                self.assertTrue(models[i].name.split('_')[-1].isdigit())

                self.assertIsInstance(models[i], SimulationModel)
                # Test pose of the model
                self.assertEqual(models[i].pose.position.tolist(), pose[0:3])
                q = Pose.rpy2quat(*pose[3::])
                diff = Pose.get_transform(models[i].pose.quat, q)

                # Test model properties
                self.assertAlmostEqual(np.sum(diff[0:3]), 0)
                self.assertFalse(models[i].static)
                self.assertEqual(len(models[i].links), 1)
                
                link_name = models[i].link_names[0]
                # Test visual element
                self.assertEqual(len(models[i].links[link_name].visuals), 1)
                geometry = models[i].links[link_name].visuals[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                                
                # Test collision element
                self.assertEqual(len(models[i].links[link_name].collisions), 1)
                geometry = models[i].links[link_name].collisions[0].geometry
                self.assertEqual(geometry.get_type(), 'sphere')
                
                # Test color exists
                material = models[i].links[link_name].visuals[0].material
                self.assertIsNotNone(material)
                self.assertIsNotNone(material.ambient)
                self.assertIsNotNone(material.diffuse)

                if not isinstance(color, str) and isinstance(color, list) and color is not None:
                    self.assertEqual(material.ambient.value, color)
                    self.assertEqual(material.diffuse.value, color)

    def test_sphere_forced_permutation(self):
        n_radius = random.randint(2, 3)
        n_masses = 2 * n_radius
        name = _get_random_string(3)

        # Test with lambda functions        
        radius = "__import__('numpy').random.random({})".format(n_radius)
        masses = "__import__('numpy').linspace(1, 10, {})".format(n_masses)

        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='sphere_factory',
                args=dict(
                    name=name,
                    radius=radius,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)
        self.assertEqual(len(models), n_radius * n_masses)

        # Test with fixed arguments
        radius = [random.random() for _ in range(n_radius)]
        masses = [random.random() for _ in range(n_masses)]

        pose = [random.random() for _ in range(6)]
        model_config = [
            dict(
                type='sphere_factory',
                args=dict(
                    name=name,
                    radius=radius,
                    mass=masses,
                    pose=pose,
                    use_permutation=False,
                    color=None
                ))
        ]

        models = create_models_from_config(model_config)
        self.assertEqual(len(models), n_radius * n_masses)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_model_factory', TestModelFactory)
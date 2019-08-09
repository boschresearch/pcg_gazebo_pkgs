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
from __future__ import print_function
from copy import deepcopy
from .properties import Pose, Inertial, Footprint, Plugin
from .link import Link
from .joint import Joint
from .sensors import IMU, Ray, Contact, Camera
from ..parsers.sdf import create_sdf_element, is_sdf_element
from ..parsers import sdf2urdf
from ..log import PCG_ROOT_LOGGER
from geometry_msgs.msg import TransformStamped
import collections
import numpy as np
from math import pi


# FIXME: Add parsing of light sources and actors
class SimulationModel(object):
    def __init__(self, name='model', parent='world', creation_time=None, 
        life_timeout=None, is_ground_plane=False):        
        self._life_timeout = life_timeout
        self._creation_time = creation_time

        self._logger = PCG_ROOT_LOGGER

        self._properties = dict()
        # Name under which the model will be spawned
        self._name = name
        # Pose of the model wrt to world
        self._pose = Pose()
        # If true, model will be static in the simulation
        self._static = False
        self._allow_auto_disable = False
        self._self_collide = False
        # Table of joints
        self._joints = dict()
        # Table of links
        self._links = dict()
        # Table of nested models
        self._models = dict()
        # If true, this model can be found in the ROS_PATH or in 
        # $HOME/.gazebo/models and can have its SDF file loaded from there
        self._is_gazebo_model = False
        # Name of the source model, in case the model's name and its 
        # source do not match
        self._source_model_name = None
        # Name of the parent model
        self._parent = parent
        # Flag to indicate if the model is a ground plane 
        self._is_ground_plane = is_ground_plane
        # List of plugins
        self._plugins = dict()

        self._logger.info('New model created, name={}'.format(self._name))

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        import sys
        if sys.version_info[0] == 2:        
            assert isinstance(value, str) or isinstance(value, unicode), \
                'Model name should be a string'
        else:
            assert isinstance(value, str), \
                'Model name should be a string'
        assert len(value) > 0, 'Model name cannot be an empty string'
        self._name = value

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        assert isinstance(value, str), 'Model parent name should be a string'
        assert len(value) > 0, 'Model parent name cannot be an empty string'
        self._parent = value

    @property
    def is_gazebo_model(self):
        return self._is_gazebo_model

    @is_gazebo_model.setter
    def is_gazebo_model(self, flag):
        assert isinstance(flag, bool), 'Flag must be a boolean'
        self._is_gazebo_model = flag

    @property
    def is_ground_plane(self):
        return self._is_ground_plane

    @is_ground_plane.setter
    def is_ground_plane(self, flag):
        assert isinstance(flag, bool), 'Input must be a boolean'
        self._is_ground_plane = flag

    @property
    def source_model_name(self):
        return self._name if self._source_model_name is None else self._source_model_name

    @property
    def static(self):
        return self._static
    
    @static.setter
    def static(self, flag):
        assert isinstance(flag, bool) or (flag in [0, 1]), \
            'Invalid boolean static flag'
        self._static = bool(flag)

    @property
    def self_collide(self):
        return self._self_collide
    
    @self_collide.setter
    def self_collide(self, flag):
        assert isinstance(flag, bool) or (flag in [0, 1]), \
            'Invalid boolean self_collide flag'
        self._self_collide = bool(flag)

    @property
    def allow_auto_disable(self):
        return self._allow_auto_disable
    
    @allow_auto_disable.setter
    def allow_auto_disable(self, flag):
        assert isinstance(flag, bool) or (flag in [0, 1]), \
            'Invalid boolean static flag'
        self._allow_auto_disable = bool(flag)

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert isinstance(vec, collections.Iterable), \
                'Input pose vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qx, qy, qz, qw)'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'All elements in pose vector must be a float or an integer'        
            if len(vec) == 6:
                self._pose = Pose(pos=vec[0:3], rpy=vec[3::])
            else:
                self._pose = Pose(pos=vec[0:3], quat=vec[3::])

    @property
    def links(self):
        return self._links

    @property
    def link_names(self):
        return self._links.keys()

    @property
    def joints(self):
        return self._joints

    @property
    def models(self):
        return self._models

    @property
    def model_names(self):
        return self._models.keys()

    @property
    def joint_names(self):
        return self._joints.keys()

    @property
    def plugins(self):
        return self._plugins

    def copy(self):
        model = SimulationModel.from_sdf(self.to_sdf())
        model.static = self.static
        model.is_gazebo_model = self.is_gazebo_model
        model._source_model_name = self._source_model_name
        return model

    def merge(self, model):
        for tag in model.links:            
            self.add_link(tag, link=model.links[tag])
            pose = model.pose + self.get_link_by_name(tag).pose
            self.get_link_by_name(tag).pose = pose
            
        for tag in model.joints:
            self.add_joint(tag, joint=model.joints[tag])

        for tag in model.plugins:
            self.add_plugin(tag, plugin=model.plugins[tag])

    def set_random_orientation(self):        
        self._pose.quat = Pose.random_quaternion()

    def get_link_by_name(self, name):
        if name not in self.links:
            return None
        return self.links[name]

    def get_joint_by_name(self, name):
        if name not in self.joints:
            return None
        return self.joints[name]  

    def get_model_by_name(self, name):
        if name not in self.models:
            return None
        return self.models[name]

    def add_cuboid_link(self, link_name=None, joint_name=None, mass=0.001, 
        size=[0.001, 0.001, 0.001], add_visual=True, mesh_filename=None, 
        mesh_scale=[1, 1, 1], add_collision=True, parent=None, joint_type='fixed',
        pose=[0, 0, 0, 0, 0, 0], color=None):        
        assert isinstance(link_name, str), 'Link name must be a string'
        assert isinstance(size, collections.Iterable), 'Size must be an array'
        assert len(list(size)) == 3, 'Input size must have 3 elements'

        self._logger.info('[{}] Adding cuboid link {}'.format(
            self.name, link_name))
        
        if joint_name is not None:
            if not isinstance(joint_name, str):
                msg = '[{}] Joint name must be a string, provided={}'.format(
                    self.name, joint_name)
                self._logger.info(msg)
                raise ValueError(msg)

        if link_name in self._links:
            self._logger.error('[{}] Link with name {} already exist'.format(
                self.name, link_name))
            return False

        if joint_name is not None:
            if parent is not None and joint_name in self._joints:
                self._logger.error('[{}] Joint with name {} already exists'.format(
                    self.name, joint_name))
                return False

        link = Link(name=link_name)
        if mass > 0:            
            link.inertial = Inertial.create_cuboid_inertia(mass, *size)
            self._logger.info('[{}] Setting mass={}, size={}, link={}'.format(
                self.name, mass, size, link_name))
            
        link.pose = pose
        self._logger.info('[{}] Link {} pose={}'.format(self.name, link_name, pose))

        if add_visual:
            link.enable_visual()
            link.add_empty_visual(name='visual')
            if mesh_filename is None:
                link.get_visual_by_name('visual').set_box_as_geometry(size=size)
                self._logger.info('[{}] Creating box visual geometry, size={}'.format(
                    self.name, size))
            else:
                link.get_visual_by_name('visual').set_mesh_as_geometry(
                    uri=mesh_filename, scale=mesh_scale)
                self._logger.info('[{}] Setting visual mesh to cuboid link {}, filename={}'.format(
                    self.name, link_name, mesh_filename))

            if color is not None:
                if color == 'random':
                    link.get_visual_by_name('visual').set_color()
                    self._logger.info('[{}] Setting random color={}, link_name={}'.format(
                        self.name, link.get_visual_by_name('visual').material.diffuse.value,
                        link_name))
                elif color == 'xkcd':
                    link.get_visual_by_name('visual').set_xkcd_color()
                    self._logger.info('[{}] Setting random xkcd color={}, link_name={}'.format(
                        self.name, link.get_visual_by_name('visual').material.diffuse.value,
                        link_name))
                elif isinstance(color, str):
                    link.get_visual_by_name('visual').set_xkcd_color(color)
                    self._logger.info('[{}] Setting xkcd color={}, link_name={}'.format(
                        self.name, link.get_visual_by_name('visual').material.diffuse.value,
                        link_name))
                elif isinstance(color, collections.Iterable) and len(list(color)) == 4:
                    link.get_visual_by_name('visual').set_color(*color)
                    self._logger.info('[{}] Setting RGBA color={}, link_name={}'.format(
                        self.name, link.get_visual_by_name('visual').material.diffuse.value,
                        link_name))
        else:
            link.disable_visual()

        if add_collision:
            link.enable_collision()
            link.add_empty_collision(name='collision')
            link.get_collision_by_name('collision').set_box_as_geometry(size=size)
            self._logger.info('[{}] Adding box collision geometry, size={}'.format(
                self.name, size))

        if parent is not None:            
            self.add_joint(name=joint_name, parent=parent, child=link_name, joint_type=joint_type)
            self._logger.info('[{}] Added joint {}'.format(self.name, joint_name))

        self.add_link(link_name, link)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_spherical_link(self, link_name=None, joint_name=None, mass=0.001,
        radius=0.001, add_visual=True, mesh_filename=None, mesh_scale=[1, 1, 1],
        add_collision=True, parent=None, joint_type='fixed', pose=[0, 0, 0, 0, 0, 0], 
        color=None):
        assert isinstance(link_name, str), 'Link name must be a string'

        self._logger.info('Adding spherical link {} to model {}'.format(
            link_name, self.name))

        if joint_name is not None:
            if not isinstance(joint_name, str):
                msg = 'Joint name must be a string, provided={}'.format(joint_name)
                self._logger.info(msg)
                raise ValueError(msg)

        if link_name in self._links:
            self._logger.error('Link with name {} already exist in model {}'.format(
                link_name, self.name))
            return False

        if parent is not None and joint_name in self._joints:
            self._logger.error('Joint with name {} already exists in model {}'.format(
                joint_name, self.name))
            return False

        link = Link(name=link_name)
        if mass > 0:
            link.inertial = Inertial.create_solid_sphere_inertia(mass, radius)
        link.pose = pose

        if add_visual:
            link.enable_visual()
            link.add_empty_visual(name='visual')
            if mesh_filename is None:
                link.get_visual_by_name('visual').set_sphere_as_geometry(radius=radius)
            else:
                link.get_visual_by_name('visual').set_mesh_as_geometry(uri=mesh_filename, scale=mesh_scale)

            if color is not None:
                if color == 'random':
                    link.get_visual_by_name('visual').set_color()
                elif color == 'xkcd':
                    link.get_visual_by_name('visual').set_xkcd_color()
                elif isinstance(color, str) or isinstance(color, unicode):
                    link.get_visual_by_name('visual').set_xkcd_color(color)
                elif isinstance(color, collections.Iterable) and len(list(color)) == 4:
                    link.get_visual_by_name('visual').set_color(*color)
        else:
            link.disable_visual()

        if add_collision:
            link.enable_collision()
            link.add_empty_collision(name='collision')
            link.get_collision_by_name('collision').set_sphere_as_geometry(radius=radius)

        if parent is not None:            
            self.add_joint(name=joint_name, parent=parent, child=link_name, joint_type=joint_type)

        self.add_link(link_name, link)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_cylindrical_link(self, link_name=None, joint_name=None, mass=0.001,
        radius=0.001, length=0.001, add_visual=True, mesh_filename=None, 
        mesh_scale=[1, 1, 1], add_collision=True, parent=None, joint_type='fixed', 
        pose=[0, 0, 0, 0, 0, 0], color=None):
        assert isinstance(link_name, str), 'Link name must be a string'

        self._logger.info('Adding cylindrical link {} to model {}'.format(
            link_name, self.name))

        if joint_name is not None:
            if not isinstance(joint_name, str):
                msg = 'Joint name must be a string, provided={}'.format(joint_name)
                self._logger.info(msg)
                raise ValueError(msg)

        if link_name in self._links:
            self._logger.error('Link with name {} already exist in model {}'.format(
                link_name, self.name))
            return False

        if parent is not None and joint_name in self._joints:
            self._logger.error('Joint with name {} already exists in model {}'.format(
                joint_name, self.name))
            return False

        link = Link(name=link_name)
        if mass > 0:
            link.inertial = Inertial.create_solid_cylinder_inertia(
                mass, radius, length, axis=[0, 0, 1])
        link.pose = pose

        if add_visual:
            link.enable_visual()
            link.add_empty_visual(name='visual')
            if mesh_filename is None:
                link.get_visual_by_name('visual').set_cylinder_as_geometry(
                    radius=radius, length=length)
            else:
                link.get_visual_by_name('visual').set_mesh_as_geometry(
                    uri=mesh_filename, scale=mesh_scale)

            if color is not None:
                if color == 'random':
                    link.get_visual_by_name('visual').set_color()
                elif color == 'xkcd':
                    link.get_visual_by_name('visual').set_xkcd_color()
                elif isinstance(color, str) or isinstance(color, unicode):
                    link.get_visual_by_name('visual').set_xkcd_color(color)
                elif isinstance(color, collections.Iterable) and len(list(color)) == 4:
                    link.get_visual_by_name('visual').set_color(*color)
        else:
            link.disable_visual()

        if add_collision:
            link.enable_collision()
            link.add_empty_collision(name='collision')
            link.get_collision_by_name('collision').set_cylinder_as_geometry(
                    radius=radius, length=length)

        if parent is not None:            
            self.add_joint(name=joint_name, parent=parent, child=link_name, joint_type=joint_type)

        self.add_link(link_name, link)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True


    def add_link(self, name='link', link=None, visual_mesh_filename=None, 
        collision_mesh_filename=None, use_approximated_collision=False, 
        approximated_collision_model='box', visual_mesh_scale=[1, 1, 1], 
        collision_mesh_scale=[1, 1, 1], pose=[0, 0, 0, 0, 0, 0], 
        color=None, mass=0, inertia=None, use_approximated_inertia=True, 
        approximated_inertia_model='box'):
        if name in self.links:
            self._logger.error('Link with name {} already exists'.format(name))
            return False

        if link is None:
            self._logger.info('Creating a new link, model_name={}, link_name={}'.format(
                self.name, name))
            link = Link(name=name)

            if visual_mesh_filename is not None:
                self._logger.info('Creating a link with the meshes provided')
                link = Link.create_link_from_mesh(
                    name=name, 
                    visual_mesh_filename=visual_mesh_filename, 
                    collision_mesh_filename=collision_mesh_filename, 
                    use_approximated_collision=use_approximated_collision, 
                    approximated_collision_model=approximated_collision_model, 
                    visual_mesh_scale=visual_mesh_scale, 
                    collision_mesh_scale=collision_mesh_scale, 
                    pose=pose, 
                    color=color, 
                    mass=mass, 
                    inertia=inertia, 
                    use_approximated_inertia=use_approximated_inertia, 
                    approximated_inertia_model=approximated_inertia_model)
        else:
            assert isinstance(link, Link), \
                'Input link is not a valid simulation object, provided={}'.format(link)
            self._logger.info('Link structure already provided')
            link.name = name
        self._links[name] = link
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_model(self, name, model=None):
        if name in self.models:
            self._logger.error('Nested model with name {} already exists'.format(name))
            return False

        if model is None:
            model = SimulationModel(name=name)
        else:
            assert isinstance(model, SimulationModel), \
                'Input model is not a valid simulation model'
            model.name = name
        model.parent = self._name
        self._models[name] = model
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_joint(self, name, parent='', child='', joint_type='', axis_limits=dict(), 
        axis_xyz=None, axis_dynamics=dict(), joint=None):
        if name in self._joints:
            self._logger.error('Joint with name {} already exists'.format(name))
            return False
        
        if joint is None:
            joint = Joint(name, parent, child, joint_type)
            if len(axis_limits):
                joint.set_axis_limits(**axis_limits)
            if len(axis_dynamics):
                joint.set_axis_dynamics(**axis_dynamics)
            if axis_xyz is not None:
                joint.set_axis_xyz(axis_xyz)
            
        self._joints[name] = joint
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True
        
    def add_contact_sensor(self, name='contact', link_name=None, joint_name=None, 
        always_on=True, update_rate=50, visualize=False, topic='/bumper', 
        pose=[0, 0, 0, 0, 0, 0], mass=0.001, add_visual=True, 
        mesh_filename=None, mesh_scale=[1, 1, 1], add_collision=True, 
        parent=None, robot_namespace='', add_ros_plugin=True,
        collision_body_name=None, link_shape='cuboid', **kwargs):
        if link_name is None:
            link_name = name + '_link'
        if joint_name is None:
            joint_name = name + '_joint'

        if link_name not in self._links:
            if link_shape == 'cuboid':
                assert 'size' in kwargs, 'Cuboid size was not provided'
                if not self.add_cuboid_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    size=kwargs['size'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'spherical':
                assert 'radius' in kwargs, 'Sphere radius was not provided'
                if not self.add_spherical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'cylindrical':
                assert 'radius' in kwargs, 'Cylinder radius was not provided'
                assert 'length' in kwargs, 'Cylinder length was not provided'
                if not self.add_cylindrical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], length=kwargs['length'], 
                    pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            else:
                raise AttributeError('Invalid link shape')
                                    
            collision_body_name = self._links[link_name].collisions[0].name

        sensor = Contact(
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose,
            collision_element_name=collision_body_name)
        
        if add_ros_plugin:
            sensor.add_ros_plugin(
                name=name, 
                robot_namespace=robot_namespace, 
                topic_name=topic)
        
        self._links[link_name].add_sensor(name, sensor)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_ray_sensor(self, name='ray', link_name=None, joint_name=None, 
        always_on=True, update_rate=50, visualize=False, topic='/scan', 
        pose=[0, 0, 0, 0, 0, 0], mass=0.001, add_visual=True, mesh_filename=None, 
        mesh_scale=[1, 1, 1], add_collision=True, parent=None, robot_namespace='', 
        add_ros_plugin=True, gaussian_noise=0, horizontal_samples=640, 
        horizontal_resolution=1, horizontal_min_angle=-pi / 2, 
        horizontal_max_angle=pi / 2, vertical_samples=1, vertical_resolution=1, 
        vertical_min_angle=0, vertical_max_angle=0, range_min=0.05, range_max=10, 
        range_resolution=0.001, noise_mean=0, noise_stddev=0, link_shape='cuboid',
        **kwargs):
        if link_name is None:
            link_name = name + '_link'
        if joint_name is None:
            joint_name = name + '_joint'

        if link_name not in self._links:
            if link_shape == 'cuboid':
                assert 'size' in kwargs, 'Cuboid size was not provided'
                if not self.add_cuboid_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    size=kwargs['size'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'spherical':
                assert 'radius' in kwargs, 'Sphere radius was not provided'
                if not self.add_spherical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'cylindrical':
                assert 'radius' in kwargs, 'Cylinder radius was not provided'
                assert 'length' in kwargs, 'Cylinder length was not provided'
                if not self.add_cylindrical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], length=kwargs['length'], 
                    pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            else:
                raise AttributeError('Invalid link shape')
        sensor = Ray(
            name=name, 
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose,
            horizontal_samples=horizontal_samples, 
            horizontal_resolution=horizontal_resolution,
            horizontal_min_angle=horizontal_min_angle, 
            horizontal_max_angle=horizontal_max_angle, 
            vertical_samples=vertical_samples, 
            vertical_resolution=vertical_resolution, 
            vertical_min_angle=vertical_min_angle, 
            vertical_max_angle=vertical_max_angle,
            range_min=range_min, 
            range_max=range_max, 
            range_resolution=range_resolution, 
            noise_mean=noise_mean, 
            noise_stddev=noise_stddev)

        if add_ros_plugin:
            sensor.add_ros_plugin(name=name, frame_name=link_name, topic_name=topic)

        self._links[link_name].add_sensor(name, sensor)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_imu_sensor(self, name='imu', link_name=None, joint_name=None, 
        always_on=True, update_rate=50, visualize=False, topic='/imu', 
        pose=[0, 0, 0, 0, 0, 0], mass=0.001, add_visual=True, mesh_filename=None, 
        mesh_scale=[1, 1, 1], add_collision=True, parent=None, noise_models=list(), 
        robot_namespace='', add_ros_plugin=True, gaussian_noise=0, 
        link_shape='cuboid', **kwargs):

        if link_name is None:
            link_name = name + '_link'
        if joint_name is None:
            joint_name = name + '_joint'

        if link_name not in self._links:
            if link_shape == 'cuboid':
                assert 'size' in kwargs, 'Cuboid size was not provided'
                if not self.add_cuboid_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    size=kwargs['size'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'spherical':
                assert 'radius' in kwargs, 'Sphere radius was not provided'
                if not self.add_spherical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'cylindrical':
                assert 'radius' in kwargs, 'Cylinder radius was not provided'
                assert 'length' in kwargs, 'Cylinder length was not provided'
                if not self.add_cylindrical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], length=kwargs['length'], 
                    pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            else:
                raise AttributeError('Invalid link shape')

        sensor = IMU(
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose)

        assert isinstance(noise_models, list), 'Noise models must be' \
            ' provided as a list of dictionaries'
        for noise in noise_models:
            assert isinstance(noise, dict), 'Noise model is not a dictionary'
            sensor.set_noise(**noise)

        if add_ros_plugin:
            sensor.add_ros_plugin(
                name=name,
                robot_namespace=robot_namespace,
                always_on=always_on,
                update_rate=update_rate,
                body_name=link_name,
                topic_name=topic,
                gaussian_noise=gaussian_noise)

        self._links[link_name].add_sensor(name, sensor)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_camera_sensor(self, name='camera', link_name=None, joint_name=None, 
        always_on=True, update_rate=50, visualize=False, topic='/camera', 
        pose=[0, 0, 0, 0, 0, 0], mass=0.001, add_visual=True, mesh_filename=None, 
        mesh_scale=[1, 1, 1], add_collision=True, parent=None, add_ros_plugin=True, 
        link_shape='cuboid', noise_mean=0, noise_stddev=0, horizontal_fov=1.047,
        image_width=320, image_height=240, image_format='R8G8B8', clip_near=0.1, 
        clip_far=100, distortion_k1=0, distortion_k2=0, distortion_k3=0,
        distortion_p1=0, distortion_p2=0, distortion_center=[0.5, 0.5], 
        image_topic_name='image_raw', camera_info_topic_name='camera_info',
        robot_namespace='', **kwargs):
        if link_name is None:
            link_name = name + '_link'
        if joint_name is None:
            joint_name = name + '_joint'

        if link_name not in self._links:
            if link_shape == 'cuboid':
                assert 'size' in kwargs, 'Cuboid size was not provided'
                if not self.add_cuboid_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    size=kwargs['size'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'spherical':
                assert 'radius' in kwargs, 'Sphere radius was not provided'
                if not self.add_spherical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            elif link_shape == 'cylindrical':
                assert 'radius' in kwargs, 'Cylinder radius was not provided'
                assert 'length' in kwargs, 'Cylinder length was not provided'
                if not self.add_cylindrical_link(
                    link_name=link_name, joint_name=joint_name, mass=mass,
                    radius=kwargs['radius'], length=kwargs['length'], 
                    pose=pose, add_collision=add_collision, 
                    add_visual=add_visual, mesh_filename=mesh_filename, 
                    mesh_scale=mesh_scale, parent=parent):
                    self._logger.error('Failed to create new contact sensor'
                                       ' link, link_name={}'.format(link_name))
                    return False
            else:
                raise AttributeError('Invalid link shape')

        sensor = Camera(
            name=name,
            always_on=always_on, 
            update_rate=update_rate, 
            visualize=visualize, 
            topic=topic, 
            pose=pose,
            noise_mean=noise_mean, 
            noise_stddev=noise_stddev, 
            horizontal_fov=horizontal_fov,
            image_width=image_width, 
            image_height=image_height, 
            image_format=image_format, 
            clip_near=clip_near, 
            clip_far=clip_far, 
            distortion_k1=distortion_k1, 
            distortion_k2=distortion_k2, 
            distortion_k3=distortion_k3,
            distortion_p1=distortion_p1, 
            distortion_p2=distortion_p2, 
            distortion_center=distortion_center)

        if add_ros_plugin:
            sensor.add_ros_camera_plugin(
                name=name,
                robot_namespace=robot_namespace,
                update_rate=0,
                camera_name=name,
                image_topic_name=image_topic_name,
                camera_info_topic_name=camera_info_topic_name,
                frame_name=link_name)

        self._links[link_name].add_sensor(name, sensor)
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False
        return True

    def add_plugin(self, name='', filename='', plugin=None, **kwargs):
        if plugin is None:
            self._plugins[name] = Plugin(
                name=name, 
                filename=filename)
            self._plugins[name].params = kwargs.copy()
        else:
            self._plugins[plugin.name] = plugin
        # If this model was a Gazebo model and it has been modified, it should not be
        # set as a Gazebo model in order to avoid errors when using <include> blocks
        self.is_gazebo_model = False

    def to_sdf(self, type='model', sdf_version='1.6'):
        assert type in ['model', 'sdf'], 'Output type must be either model or sdf'
        model = create_sdf_element('model')
        model.name = self._name
        model.pose = self.pose.to_sdf()
        model.static = self._static  
        model.self_collide = self._self_collide      
        model.allow_auto_disable = self._allow_auto_disable

        for tag in self.links:
            model.add_link(tag, self.links[tag].to_sdf('link'))

        for tag in self.joints:
            model.add_joint(tag, self.joints[tag].to_sdf())

        for tag in self.models:
            model.add_model(tag, self.models[tag].to_sdf())     

        for tag in self.plugins:
            model.add_plugin(tag, plugin=self.plugins[tag].to_sdf())   
        
        if type == 'model':
            return model

        sdf = create_sdf_element('sdf')
        sdf.reset('model')

        sdf.version = sdf_version
        sdf.add_model(model.name, model)

        return sdf

    @staticmethod
    def from_sdf(sdf):
        if sdf._NAME != 'model':
            msg = 'SDF element must be of type <model>'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        
        model = SimulationModel()
        # Store model name
        model.name = sdf.name
        
        # Store the value parameters
        model.self_collide = False if sdf.self_collide is None else sdf.self_collide.value
        model.allow_auto_disable = True if sdf.allow_auto_disable is None else sdf.allow_auto_disable.value
        model.static = False if sdf.static is None else sdf.static.value

        # Set model pose
        if sdf.pose is not None:
            model.pose.from_sdf(sdf.pose)
        # Parse links
        if sdf.links:
            for link_sdf in sdf.links:                
                model.add_link(
                    link_sdf.name, Link.from_sdf(link_sdf))
        # Parse joints
        if sdf.joints:
            for joint_sdf in sdf.joints:                
                model._joints[joint_sdf.name] = Joint.from_sdf(joint_sdf)
        
        # Parse nested included models
        if sdf.includes:
            for include_sdf in sdf.includes:
                model_name = include_sdf.uri.value.replace('model://', '')
                instance_name = include_sdf.name.value
                model.add_model(
                    instance_name, 
                    SimulationModel.from_gazebo_model(model_name))
                if include_sdf.pose is not None:
                    model.models[instance_name].pose = include_sdf.pose.value
                if include_sdf.static is not None:
                    model.models[instance_name].static = include_sdf.static.value
        # Parse nested models
        if sdf.models:            
            for model_sdf in sdf.models:                                
                model.add_model(
                    model_sdf.name, 
                    SimulationModel.from_sdf(model_sdf))  

        # Parse plugins
        if sdf.plugins:
            for plugin_sdf in sdf.plugins:
                model.add_plugin(
                    name=plugin_sdf.name,
                    plugin=Plugin.from_sdf(plugin_sdf))

        return model

    @staticmethod
    def from_gazebo_model(name):
        from . import get_gazebo_model_sdf
        PCG_ROOT_LOGGER.info('Importing a Gazebo model, name={}'.format(name))
        # Update list of Gazebo models
        sdf = get_gazebo_model_sdf(name)
        
        if sdf is None:
            msg = 'Gazebo model {} not found in the ROS paths'.format(name)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)        
        if sdf.models is None:
            msg = 'No models found in Gazebo model {}'.format(name)
            PCG_ROOT_LOGGER.warning(msg)
            raise ValueError(msg)
        if len(sdf.models) != 1:
            msg = 'Imported SDF file should have one model only'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        model = SimulationModel.from_sdf(sdf.models[0])
        model.is_gazebo_model = True
        model._source_model_name = name
        return model

    def to_urdf(self):
        self._logger.info('Exporting model <{}> as URDF'.format(self.name))
        model_sdf = self.to_sdf()
        return sdf2urdf(model_sdf)

    def get_tf_transforms(self):
        tfs = list()

        # Set frame IDs and timestamp
        model_transform = TransformStamped()
        model_transform.header.frame_id = self._parent
        model_transform.child_frame_id = self.get_frame_id()
        # Set pose of the model
        model_transform.transform.translation.x = self.pose.position[0]
        model_transform.transform.translation.y = self.pose.position[1]
        model_transform.transform.translation.z = self.pose.position[2]

        model_transform.transform.rotation.x = self.pose.quat.x
        model_transform.transform.rotation.y = self.pose.quat.y
        model_transform.transform.rotation.z = self.pose.quat.z
        model_transform.transform.rotation.w = self.pose.quat.w

        # Main TF for the model
        tfs.append(model_transform)

        # In case there are nested models, add them to the list
        for tag in self.models:
            tfs.append(self.models[tag].get_tf_transforms())
        return tfs

    def get_frame_id(self):
        if self._parent == 'world':
            return self._name
        else:
            return '{}/{}'.format(self._parent, self._name)            

    def to_markers(self):
        markers = list()

        for tag in self._models:  
            model_markers = self._models[tag].to_markers()    
            for i in range(len(model_markers)):
                model_markers[i].header.frame_id = self._models[tag].get_frame_id()
                model_markers[i].ns = self._models[tag].get_frame_id() + '/' + model_markers[i].ns
            markers += model_markers        

        for tag in self._links:
            link_markers = self._links[tag].to_markers()
            for i in range(len(link_markers)):                
                link_markers[i].header.frame_id = self.get_frame_id()
                link_markers[i].ns = self.get_frame_id() + '/' + link_markers[i].ns
            markers += link_markers
                       
        return markers

    def get_footprint(self, mesh_type='collision', pose_offset=None, use_bounding_box=False, 
        z_limits=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints must be either collision or visual' \
                  ' geometries, provided={}'.format(mesh_type)
            self._logger.error(msg)
            raise ValueError(msg)
        
        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                self._logger.error(msg)
                raise ValueError(msg)            
        else:
            pose_offset = Pose()

        if z_limits is not None:
            if not isinstance(z_limits, collections.Iterable):
                msg = 'Z limits input has to be a list, provided={}'.format(z_limits)
                self._logger.error(msg)
                raise ValueError(msg)

        combined_pose = pose_offset + self._pose
        
        footprints = dict()        
        for tag in self._models:               
            footprint = self._models[tag].get_footprint(
                mesh_type, combined_pose, use_bounding_box, z_limits)                     
            if footprint is not None:                                
                footprints[self.name + '::' + self._models[tag].name] = footprint        

        model_footprint = Footprint()
        for tag in self._links:
            footprint = self._links[tag].get_footprint(
                mesh_type, combined_pose, use_bounding_box, z_limits)
            if footprint is not None:
                model_footprint.add_polygon(footprint)
                        
        combined_model_footprint = model_footprint.get_footprint_polygon()
        
        if combined_model_footprint is not None:
            footprints[self.name] = combined_model_footprint

        self._logger.info('Footprint computed for model <{}>'.format(self.name))
        return footprints

    def get_meshes(self, mesh_type='collision', pose_offset=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints must be either collision or visual' \
                  ' geometries, provided={}'.format(mesh_type)
            self._logger.error(msg)
            raise ValueError(msg)

        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                self._logger.error(msg)
                raise ValueError(msg)            
        else:
            pose_offset = Pose()

        combined_pose = pose_offset + self._pose

        meshes = list()
        for tag in self._models:
            meshes = meshes + self._models[tag].get_meshes(mesh_type, combined_pose)

        for tag in self._links:
            meshes = meshes + self._links[tag].get_meshes(mesh_type, combined_pose)

        return meshes

    def create_scene(self, mesh_type='collision', add_pseudo_color=True):
        from ..visualization import create_scene
        return create_scene([self], mesh_type, add_pseudo_color)   

    def show(self, mesh_type='collision', add_pseudo_color=True):
        scene = self.create_scene(mesh_type, add_pseudo_color)     
        scene.show()

    def plot_footprint(self, fig=None, ax=None, fig_width=20, fig_height=20,
        mesh_type='collision', z_limits=None, colormap='magma', grid=True, 
        ignore_ground_plane=True, line_width=1, line_style='solid', alpha=0.5, 
        engine='matplotlib', dpi=200):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints must be either collision or visual' \
                  ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)        

        if z_limits is None:
            PCG_ROOT_LOGGER.info('Plotting footprints using Z limits: {}'.format(z_limits))

        from ..visualization import plot_footprints

        input_model = dict()
        input_model[self.name] = self
        fig = plot_footprints(
            models=input_model,
            fig=fig,
            ax=ax,
            fig_height=fig_height,
            fig_width=fig_width,
            mesh_type=mesh_type,
            engine=engine,
            line_style=line_style,
            line_width=line_width,
            grid=grid,
            ignore_ground_plane=ignore_ground_plane,
            z_limits=z_limits,
            colormap=colormap,
            dpi=dpi,
            alpha=alpha
        )

        PCG_ROOT_LOGGER.info('Plotting footprints: finished')
        return fig

    def get_bounds(self, mesh_type='collision'):
        meshes = self.get_meshes(mesh_type)

        bounds = None
        for mesh in meshes:
            if bounds is None:
                bounds = deepcopy(mesh.bounds)
            else:
                cur_bounds = deepcopy(mesh.bounds)
                for i in range(3):
                    bounds[0, i] = min(bounds[0, i], cur_bounds[0, i])
                for i in range(3):
                    bounds[1, i] = max(bounds[1, i], cur_bounds[1, i])
        return bounds

    def spawn(self, gazebo_proxy=None, robot_namespace=None, pos=[0, 0, 0], 
        rot=[0, 0, 0], reference_frame='world', timeout=30, replace=True):
        from ..task_manager import GazeboProxy, is_gazebo_running        
        from time import sleep, time

        if not isinstance(gazebo_proxy, GazeboProxy):
            gazebo_proxy = GazeboProxy()

        if robot_namespace is None:
            robot_namespace = self.name

        assert timeout >= 0, 'Timeout should be equal or greater than zero'
        start_time = time()
        while not gazebo_proxy.is_init() and time() - start_time < timeout:
            self._logger.info('Waiting for Gazebo to start...')
            sleep(0.5)

        if not is_gazebo_running(
            ros_master_uri=gazebo_proxy.ros_config.ros_master_uri):
            self._logger.error('Gazebo is not running!')
            return False

        if replace and robot_namespace in gazebo_proxy.get_model_names():
            self._logger.info('Deleting existing model first')
            if gazebo_proxy.delete_model(robot_namespace):
                self._logger.info('Done')
            else:
                self._logger.error('Failed to delete existing model')
                return False

        sdf = self.to_sdf(type='model')
        assert sdf._NAME == 'model', 'SDF element must be of type model'
        assert is_sdf_element(sdf), 'Input is not an SDF element'
        sdf_root = create_sdf_element('sdf')
        sdf_root.reset(mode='model')
        sdf_root.add_model(model=sdf)
        
        return gazebo_proxy.spawn_sdf_model(
            robot_namespace,
            sdf_root.to_xml_as_str(),
            pos,
            rot,
            reference_frame)
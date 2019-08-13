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
import collections
from copy import deepcopy
from .properties import Pose
from .model import SimulationModel
from .light import Light
from ..log import PCG_ROOT_LOGGER


class ModelGroup(object):
    def __init__(self, name='group', pose=[0, 0, 0, 0, 0, 0], 
        is_ground_plane=False):
        self._name = ''
        self._pose = Pose()    
        self._models = dict()
        self._lights = dict()
        # Flag to indicate if the model is a ground plane 
        self._is_ground_plane = is_ground_plane

        # Set model group input parameters
        self.name = name
        self.pose = pose

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
    def prefix(self):
        prefix = ''
        if self.name != 'default':
            prefix = '{}/'.format(self.name)
        return prefix

    @property
    def is_ground_plane(self):
        return self._is_ground_plane

    @is_ground_plane.setter
    def is_ground_plane(self, flag):
        assert isinstance(flag, bool), 'Input must be a boolean'
        self._is_ground_plane = flag

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
    def models(self):
        """`dict`: Models"""        
        return self._models

    @property
    def lights(self):
        """`dict`: Lights"""
        return self._lights

    @property
    def n_models(self):
        """`int`: Number of models"""
        n_models = 0
        for tag in self._models:
            if isinstance(self._models[tag], SimulationModel):
                n_models += 1
            else:
                n_models += self._models[tag].n_models
        return n_models

    @property
    def n_lights(self):
        """`int`: Number of lights"""
        return len(self._lights)

    def reset_models(self):
        """Reset the list of models."""
        self._models = dict()

    def add_model(self, tag, model):
        """Add a model to the world.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Model's local name in the world. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being 
        an integer.
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): 
        Model object
        
        > *Returns*
        
        `bool`: `True`, if model could be added to the world.
        """
        assert isinstance(model, SimulationModel) or isinstance(model, ModelGroup), \
            'Input model is not of type SimulationModel nor a ModelGroup'        
        if self.model_exists(tag):
            # Add counter suffix to add models with same name
            i = 0
            new_model_name = '{}'.format(tag)
            while self.model_exists(new_model_name):
                i += 1
                new_model_name = '{}_{}'.format(tag, i)
            name = new_model_name
        else:
            name = tag
        
        self._models[name] = model        
        self._models[name].name = name
        return name   
        
    def rm_model(self, tag):
        """Remove model from world.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Local name identifier of the 
        model to be removed.
        
        > *Returns*
        
        `bool`: `True`, if model could be removed, `False` if 
        no model with name `tag` could be found in the world.
        """
        if tag in self._models:
            del self._models[tag]
            return True
        return False

    def model_exists(self, tag):
        """Test if a model with name `tag` exists in the world description.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Local name identifier of the model.
        
        > *Returns*
        
        `bool`: `True`, if model exists, `False`, otherwise.
        """
        return tag in self._models

    def add_include(self, include):
        """Add a model via include method.
        
        > *Input arguments*
        
        * `include` (*type:* `pcg_gazebo.parsers.sdf.Include`): 
        SDF `<include>` element
        
        > *Returns*
        
        `bool`: `True`, if model directed by the `include` element
        could be parsed and added to the world.
        """
        from . import get_gazebo_model_sdf
        model_name = include.uri.value.replace('model://', '')    
        try:    
            model = SimulationModel.from_gazebo_model(model_name)
            # Set model pose, if specified
            if include.pose is not None:
                model.pose = include.pose.value
            # Set the model as static, if specified
            if include.static is not None:
                model.static = include.static.value
            if include.name is not None:
                name = include.name.value
            else:   
                name = model_name
            return self.add_model(name, model)
        except ValueError as ex:
            sdf = get_gazebo_model_sdf(model_name)
            if sdf.lights is not None:
                PCG_ROOT_LOGGER.info('Loading model {} as light source model'.format(
                    model_name))        
                for light in sdf.lights:
                    light = Light.from_sdf(light)
                    self.add_light(light.name, light)
                    PCG_ROOT_LOGGER.info('Added light {} from included model {}'.format(
                        light.name, model_name))        

    def get_meshes(self, mesh_type='collision', pose_offset=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints must be either collision or visual' \
                  ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)            
        else:
            pose_offset = Pose()

        combined_pose = pose_offset + self._pose

        meshes = list()
        for tag in self._models:
            meshes = meshes + self._models[tag].get_meshes(mesh_type, combined_pose)

        return meshes

    def get_footprint(self, mesh_type='collision', pose_offset=None, use_bounding_box=False, 
        z_limits=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints must be either collision or visual' \
                  ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        
        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)            
        else:
            pose_offset = Pose()

        if z_limits is not None:
            if not isinstance(z_limits, collections.Iterable):
                msg = 'Z limits input has to be a list, provided={}'.format(z_limits)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

        combined_pose = pose_offset + self._pose
        
        footprints = dict()        
        for tag in self._models:               
            footprint = self._models[tag].get_footprint(
                mesh_type, combined_pose, use_bounding_box, z_limits)                     
            if footprint is not None:                                
                footprints[self.name + '::' + self._models[tag].name] = footprint                

        PCG_ROOT_LOGGER.info('Footprint computed for model <{}>'.format(self.name))
        return footprints

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

    def create_scene(self, mesh_type='collision', add_pseudo_color=True):
        from ..visualization import create_scene
        return create_scene(self.get_models(with_group_prefix=True).values(), mesh_type, add_pseudo_color)   

    def show(self, mesh_type='collision', add_pseudo_color=True):
        scene = self.create_scene(mesh_type, add_pseudo_color)     
        scene.show()

    def get_model(self, name, with_group_prefix=True):
        prefix = self.prefix if with_group_prefix else ''
        if '/' not in name:
            if name not in self._models:
                PCG_ROOT_LOGGER.warning('No model {} found in group {}'.format(
                    name, self.name))
                return None
                
            output = self._models[name].copy()
            output.pose = output.pose + self._pose                        
            output.name = prefix + output.name
        else:
            sub_group_name = name.split('/')[0]
            if sub_group_name not in self._models:
                PCG_ROOT_LOGGER.warning('<{}> is not a model group in <{}>'.format(
                    sub_group_name, self.name))
                return None
            output = self._models[sub_group_name].get_model(
                name=name.replace(sub_group_name + '/', ''), 
                with_group_prefix=True)
            output.pose = output.pose + self._pose                
            output.name = prefix + output.name
        PCG_ROOT_LOGGER.info('Retrieving model <{}> from group <{}>'.format(
            output.name, self.name))
        return output

    def get_models(self, with_group_prefix=True):
        prefix = self.prefix if with_group_prefix else ''
        output_models = dict()
        for name in self._models:
            if isinstance(self._models[name], SimulationModel):
                model = self.get_model(name, with_group_prefix)
                output_models[model.name] = self.get_model(name, with_group_prefix)
            elif isinstance(self._models[name], ModelGroup):
                models = self._models[name].get_models(with_group_prefix=True)
                for name in models:
                    models[name].name = prefix + models[name].name
                    models[name].pose = models[name].pose + self._pose
                    output_models[models[name].name] = models[name]
        return output_models

    def get_light(self, name, with_group_prefix=True):
        prefix = self.prefix if with_group_prefix else ''
        light = None
        if '/' not in name:
            if name not in self._lights:
                PCG_ROOT_LOGGER.warning('No light model <{}> found in group {}'.format(
                    name, self.name))
                return None            

            light = self._lights[name].copy()
            light.pose = light.pose + self._pose                
            light.name = prefix + light.name            
        else:
            sub_group_name = name.split('/')[0]
            if sub_group_name not in self._models:
                PCG_ROOT_LOGGER.warning('<{}> is not a model group in <{}>'.format(
                    sub_group_name, self.name))
                return None
            light = self._models[sub_group_name].get_light(
                name=name.replace(sub_group_name + '/', ''), 
                with_group_prefix=True)
            light.pose = light.pose + self._pose                
            light.name = prefix + light.name
        PCG_ROOT_LOGGER.info('Retrieving light model <{}> from group <{}>'.format(
            light.name, self.name))
        return light

    def get_lights(self, with_group_prefix=True):
        prefix = self.prefix if with_group_prefix else ''
        output_lights = dict()        
        for name in self._lights:          
            light = self.get_light(name, with_group_prefix)
            output_lights[light.name] = self.get_light(name, with_group_prefix)
        for tag in self._models:
            if isinstance(self._models[tag], ModelGroup):
                lights = self._models[tag].get_lights(with_group_prefix=True)
                for name in lights:
                    lights[name].name = prefix + lights[name].name
                    lights[name].pose = lights[name].pose + self._pose
                    output_lights[lights[name].name] = lights[name]                
        return output_lights

    def add_light(self, tag, light):
        """Add light description to the world.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Name identifier for the plugin. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being 
        an integer.
        * `light` (*type:* `pcg_gazebo.parsers.sdf.Light` or 
        `pcg_gazebo.simulation.properties.Light`): Light description
        """
        assert isinstance(light, Light), 'Invalid light object'
        if self.light_exists(tag):
            # Add counter suffix to add lights with same name
            i = 0
            new_light_name = '{}'.format(tag)
            while self.light_exists(new_light_name):
                i += 1
                new_light_name = '{}_{}'.format(tag, i)
            name = new_light_name
        else:
            name = tag
        
        self._lights[name] = light
        self._lights[name].name = name
        PCG_ROOT_LOGGER.info('Light added, name={}, group={}'.format(
            name, self.name))
        return True
        
    def rm_light(self, tag):
        """Remove light from world.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Local name identifier of the 
        light to be removed.
        
        > *Returns*
        
        `bool`: `True`, if light could be removed, `False` if 
        no light with name `tag` could be found in the world.
        """
        if tag in self._lights:
            del self._lights[tag]
            PCG_ROOT_LOGGER.info('Light removed, name={}, group={}'.format(
                tag, self.name))
            return True
        return False

    def light_exists(self, tag):
        """Test if a light with name `tag` exists in the world description.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Local name identifier of the light.
        
        > *Returns*
        
        `bool`: `True`, if light exists, `False`, otherwise.
        """
        return tag in self._lights

    def to_sdf(self, use_include=True):
        from . import is_gazebo_model
        from ..parsers.sdf import create_sdf_element
        sdf_models = dict()
        sdf_lights = dict()
        sdf_includes = dict()

        models = self.get_models(with_group_prefix=True)
        for name in models:            
            if (models[name].is_gazebo_model or is_gazebo_model(name)) and use_include:
                include = create_sdf_element('include')
                include.uri = 'model://' + models[name]._source_model_name                
                include.pose = list(models[name].pose.position) + list(models[name].pose.rpy)
                include.name = name
                include.static = models[name].static
                sdf_includes[name] = include
            else:
                sdf_models[name] = models[name].to_sdf()
            
        lights = self.get_lights(with_group_prefix=True)
        for name in lights:
            sdf_lights[name] = lights[name].to_sdf()

        return sdf_models, sdf_lights, sdf_includes

    @staticmethod
    def from_sdf(sdf):
        group = None
        if isinstance(sdf, list):
            group = ModelGroup()
            for elem in sdf:
                if elem.xml_element_name == 'model':
                    model = SimulationModel.from_sdf(elem)
                    if model is None:
                        PCG_ROOT_LOGGER.error('Failed to load model={}'.format(elem))
                    else:
                        group.add_model(model.name, model)   
                elif elem.xml_element_name == 'light':
                    light = Light.from_sdf(elem)
                    if light is None:
                        PCG_ROOT_LOGGER.error('Failed to load light={}'.format(elem))
                    else:
                        group.add_light(light.name, light)
                elif elem.xml_element_name == 'include':
                    group.add_include(elem)
                elif elem.xml_element_name == 'actor':
                    PCG_ROOT_LOGGER.warning('Adding actors not implemented yet')                
        elif sdf.xml_element_name == 'sdf':
            group = ModelGroup()
            if sdf.models is not None:
                for elem in sdf.models:
                    model = SimulationModel.from_sdf(elem)
                    if model is None:
                        PCG_ROOT_LOGGER.error('Failed to load model={}'.format(elem))
                    else:
                        group.add_model(model.name, model)   

            if sdf.lights is not None:
                for elem in sdf.lights:
                    light = Light.from_sdf(elem)
                    if light is None:
                        PCG_ROOT_LOGGER.error('Failed to load light={}'.format(elem))
                    else:
                        group.add_light(light.name, light)

            # TODO Add load actors function
                
        return group

    @staticmethod
    def from_gazebo_model(name):
        from . import get_gazebo_model_sdf
        PCG_ROOT_LOGGER.info('Importing a Gazebo model, name={}'.format(name))
        sdf = get_gazebo_model_sdf(name)        
        return ModelGroup.from_sdf(sdf)

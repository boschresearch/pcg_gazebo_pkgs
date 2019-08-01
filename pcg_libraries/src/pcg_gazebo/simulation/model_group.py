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
from ..log import PCG_ROOT_LOGGER


class ModelGroup(object):
    def __init__(self, name='group', pose=[0, 0, 0, 0, 0, 0]):
        self._name = ''
        self._pose = Pose()    
        self._models = dict()

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
    def n_models(self):
        """`int`: Number of models"""
        return len(self._models)

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
        * `model` (*type:* `pcg_gazebo.simulaton.SimulationModel`): 
        Model object
        
        > *Returns*
        
        `bool`: `True`, if model could be added to the world.
        """
        assert isinstance(model, SimulationModel), \
            'Input model is not of type SimulationModel'
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
        return True   
        
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
        return create_scene(self._models.values(), mesh_type, add_pseudo_color)   

    def show(self, mesh_type='collision', add_pseudo_color=True):
        scene = self.create_scene(mesh_type, add_pseudo_color)     
        scene.show()


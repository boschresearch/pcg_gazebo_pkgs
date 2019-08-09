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
from ..simulation import SimulationModel, ModelGroup, Light
from ._collection_manager import _CollectionManager
from ..log import PCG_ROOT_LOGGER


class AssetsManager(_CollectionManager):
    def __init__(self):
        super(AssetsManager, self).__init__()
        self._ground_plane_assets = list()
    
    @staticmethod
    def get_instance():
        if AssetsManager._INSTANCE is None:
            AssetsManager._INSTANCE = AssetsManager()
        return AssetsManager._INSTANCE

    @property
    def tags(self):
        from ..simulation import get_gazebo_model_names
        return list(self._collection.keys()) + list(get_gazebo_model_names())

    @property
    def ground_planes(self):
        return self._ground_plane_assets

    def is_model(self, tag):
        if tag in self._collection:
            return isinstance(self._collection[tag], SimulationModel)
        return False

    def is_light(self, tag):
        if tag in self._collection:
            return isinstance(self._collection[tag], Light)
        return False
    
    def is_model_group(self, tag):
        if tag in self._collection:
            return isinstance(self._collection[tag], ModelGroup)
        return False

    def is_gazebo_model(self, tag):
        from ..simulation import is_gazebo_model
        return is_gazebo_model(tag)

    def is_model_group_generator(self, tag):
        from .model_group_generator import ModelGroupGenerator
        if tag in self._collection:
            return isinstance(self._collection[tag], ModelGroupGenerator)
        return False

    def is_ground_plane(self, tag):
        return tag in self._ground_plane_assets

    def is_factory_input(self, tag):
        if self.has_element(tag):
            if not isinstance(self._collection[tag], dict):
                return False
            if 'type' not in self._collection[tag] or 'args' not in self._collection[tag]:
                return False
            if self._collection[tag]['type'] in \
                ['box', 'sphere', 'cylinder', 'mesh']:
                return True
        return False
    
    def add(self, description, tag=None, type=None, parameters=None, include_dir=None):
        import os
        from ..parsers import parse_sdf, parse_urdf, urdf2sdf
        from .model_group_generator import ModelGroupGenerator
        assert isinstance(description, SimulationModel) or \
            isinstance(description, Light) or \
            isinstance(description, ModelGroup) or \
            isinstance(description, ModelGroupGenerator) or \
            isinstance(description, dict) or isinstance(description, str)
        if hasattr(description, 'name'):
            if tag is None:
                tag = description.name                            
            self._collection[tag] = description
        elif isinstance(description, str):
            if os.path.isfile(description):
                ext = description.split('.')[-1]
                if ext == 'sdf':
                    sdf = parse_sdf(description)
                elif ext == 'urdf':
                    sdf = urdf2sdf(parse_urdf(description))
                else:
                    return False
                # TODO Add processing Jinja templates to generate assets
                if sdf.xml_element_name == 'model':
                    try:
                        model = SimulationModel.from_sdf(sdf)
                        if model is not None:
                            return self.add(model, tag=tag)
                        else:
                            PCG_ROOT_LOGGER.error('No model found in file={}'.format(description))
                            return False
                    except ValueError as ex:
                        PCG_ROOT_LOGGER.error('Failed to load a model from file={}, message={}'.format(
                            description, ex))
                        return False                    
                elif sdf.xml_element_name == 'sdf':
                    if len(sdf.models) == 1 and len(sdf.lights) == 0:
                        return self.add(SimulationModel.from_sdf(sdf.models[0]), tag=tag)
                    elif len(sdf.models) == 0 and len(sdf.lights) == 1:                                         
                        return self.add(Light.from_sdf(sdf.lights[0]), tag=tag)
                    elif len(sdf.models) > 0 or len(sdf.lights) > 0:
                        return self.add(ModelGroup.from_sdf(sdf), tag=tag)
            else:
                # The string must be otherwise an already existant
                # Gazebo model
                if not self.is_gazebo_model(tag):
                    PCG_ROOT_LOGGER.error('Input string does not refer to a Gazebo'
                    ' model, value={}'.format(description))
                    return False
                self._collection[tag] = description
        else:
            if type is None or type == 'factory':
                if not isinstance(description, dict):
                    PCG_ROOT_LOGGER.error(
                        'Factory model constructor must be a dict, value={}'.format(
                        description))
                    return False
                if 'type' not in description or 'args' not in description:
                    PCG_ROOT_LOGGER.error('Factory model constructor requires '
                        'type and args inputs')
                    return False
                if description['type'] not in \
                    ['box', 'sphere', 'cylinder', 'mesh']:
                    PCG_ROOT_LOGGER.error('Type of factory model constructor must be either'
                        ' box, sphere, cylinder or mesh, received={}'.format(description['type']))
                    return False
                type = 'factory'
                self._collection[tag] = description
                PCG_ROOT_LOGGER.info('Added model factory <{}>'.format(tag))
            elif type in ['model_generator', 'light']:                
                if type == 'model_generator':
                    self._collection[tag] = ModelGroupGenerator.from_dict(description)
                    PCG_ROOT_LOGGER.info('Added model group generator <{}>'.format(tag))
                else:
                    self._collection[tag] = Light.from_dict(description)
                    PCG_ROOT_LOGGER.info('Added light <{}>'.format(tag))
            else:
                return False

        PCG_ROOT_LOGGER.info('New asset added={}'.format(tag))
        return True

    def get(self, tag, *args, **kwargs):
        model = None
        if self.is_model(tag) or self.is_light(tag):
            model = self._collection[tag].copy()
        elif self.is_model_group(tag):
            return self._collection[tag]
        elif self.is_gazebo_model(tag):
            model = None
            try:
                model = SimulationModel.from_gazebo_model(tag)
            except ValueError as ex:            
                model = ModelGroup.from_gazebo_model(tag)            
        elif self.is_model_group_generator(tag):
            model = self._collection[tag].run(group_name=tag, *args, **kwargs)    
        elif self.is_factory_input(tag):
            from .creators import config2models
            model = SimulationModel.from_sdf(config2models(self._collection[tag])[0])
            model.name = tag
        if model is not None and not self.is_light(tag):
            model.is_ground_plane = self.is_ground_plane(tag)
        return model

    def set_asset_as_ground_plane(self, tag):
        """Flag a model asset as part of the ground plane. This procedure will 
        affect the collision checks during the automatic placement of models in
        the world using the placement engines.
        
        > *Input arguments*
        
        * `tag` (*type:* `str`): Name of the model asset        
        """
        assert isinstance(tag, str), 'Input tag must be a string'
        if tag not in self._collection and not self.is_gazebo_model(tag):
            PCG_ROOT_LOGGER.error('Asset <{}> not in the list of assets'.format(tag))
            return False
        else:
            if tag not in self._ground_plane_assets:
                self._ground_plane_assets.append(tag)
            return True

    def from_dict(self, config):
        assert isinstance(config, dict), 'Input must be a dictionary'

        if 'assets' in config:
            assert isinstance(config['assets'], list), \
                'Assets must be provided as a list'
            for elem in config['assets']:
                if not self.add(**elem):
                    PCG_ROOT_LOGGER.error('Error adding asset={}'.format(elem))

        # Setting assets as ground plane
        if 'ground_plane' in config:
            assert isinstance(config['ground_plane'], list), \
                'ground_plane input for assets manager must be a list of strings'
            for tag in config['ground_plane']:
                if not self.set_asset_as_ground_plane(tag):
                    PCG_ROOT_LOGGER.error('Error setting asset <{}> as ground plane'.format(
                        tag))
            
    def from_yaml(self, filename):
        pass
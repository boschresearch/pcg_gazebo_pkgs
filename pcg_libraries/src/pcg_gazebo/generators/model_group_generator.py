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
from .engines import create_engine
from .constraints import create_constraint
from ..simulation import SimulationModel, ModelGroup
from ..log import PCG_ROOT_LOGGER


class ModelGroupGenerator(object):
    def __init__(self, name='group'):
        self._logger = PCG_ROOT_LOGGER
        
        self._name = name
        self._assets = dict()
        self._engines = list()
        self._constraints = dict()
        self._states = list()

    @property
    def assets(self):
        """List of `pcg_gazebo.simulation.SimulationModel`: List of model 
        assets that will be used of the world generation.
        """
        return self._assets

    @property
    def engines(self):
        """`dict` of `pcg_gazebo.generators.engines`: Dictionary with the
        model creation engines.
        """
        return self._engines
    
    @property
    def constraints(self):
        """`dict` of `pcg_gazebo.generators.constraints`: Dictionary with the
        positioning constraints.
        """
        return self._constraints

    def get_model_instance(self, name):
        asset = self.get_asset(name)
        if isinstance(asset, SimulationModel) or isinstance(asset, ModelGroup):
            return asset
        elif isinstance(asset, dict):
            from .creators import config2models
            model_sdfs = config2models(asset)
            if len(model_sdfs) != 1:
                PCG_ROOT_LOGGER.error(
                    'Model factory asset {} returned an invalid'
                    ' number of models, len={}'.format(name, len(model_sdfs)))
                return None
            else:
                return model_sdfs[0]
        elif callable(asset):
            model = asset()
            if not isinstance(model, SimulationModel) and not isinstance(model, ModelGroup):
                PCG_ROOT_LOGGER.error(
                    'Model lambda function {} did not return'
                    ' a model or a model group, retrieved={}'.format(name, type(model)))
            return model                

    def add_engine(self, engine_name, models, **kwargs):
        """Add a new model creator engine to the internal engines list.
        
        > *Input arguments*
        
        * `engine_name` (*type:* `str`): Name of the engine class to be created
        * `models` (*type:* list of `str`): Name of the models that will be assets 
        to the created engine
        * `kwargs` (*type:* `dict`): Input arguments to the created engine.        
        """
        input_args = kwargs
        input_args['models'] = models      
        input_args['callback_fcn_get_model'] = self.get_model_instance
        input_args['callback_fcn_get_constraint'] = self.get_constraint
        self._engines.append(create_engine(engine_name, **input_args))
        self._logger.info('New model creator engine added, type={}'.format(engine_name))
        self._logger.info(self._engines[-1].__str__())

    def set_model_as_ground_plane(self, model_name):
        """Flag a model asset as part of the ground plane. This procedure will 
        affect the collision checks during the automatic placement of models in
        the world using the placement engines.
        
        > *Input arguments*
        
        * `model_name` (*type:* `str`): Name of the model asset        
        """
        if model_name not in self._assets:
            self._logger.error('Model {} no in the list of assets'.format(model_name))
        self._assets[model_name].is_ground_plane = True

    def get_asset(self, name):
        """Return a simulation model asset.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Name of the model asset.
        
        > *Returns*
        
        The model asset as `pcg_gazebo.simulation.SimulationModel`. 
        `None` if `name` cannot be found in the list of model assets.
        """
        from ..simulation import is_gazebo_model, get_gazebo_model_sdf
        if name not in self._assets:
            if is_gazebo_model(name):
                self._assets[name] = get_gazebo_model_sdf(name)
            else:
                PCG_ROOT_LOGGER.error(
                    'Requested asset {} is neither already in the'
                    ' assets list nor a Gazebo model in the ROS path'.format(name))
                return None
        return self._assets[name]

    def remove_asset(self, name):
        """Remove model asset from the list of assets.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Name of the model
        
        > *Returns*
        
        `True`, if model could be removed.
        """
        if name in self._assets:
            del self._assets[name]
            PCG_ROOT_LOGGER.info('Asset {} was successfully deleted'.format(name))
            return True
        return False

    def get_constraint(self, name):
        """Return a positioning constraint configuration.
        
        > *Input arguments*
        
        * `param` (*type:* `data_type`, *default:* `data`): Parameter description
        
        > *Returns*
        
        Description of return values
        """
        if name not in self._constraints:
            PCG_ROOT_LOGGER.error('Contraint {} could not be found'.format(name))
            return None
        return self._constraints[name]
    
    def add_gazebo_model_as_asset(self, gazebo_model_name):
        """Create a model asset by importing a Gazebo model that already
        exists in the resources path of the catkin workspace. The model's
        SDF file will be parsed and converted into a `pcg_gazebo.simulation.SimulationModel`
        instance.

        Models that include lights can also be added, but will not be considered
        assets, they will just be included into the generated world SDF file.
        
        > *Input arguments*
        
        * `gazebo_model_name` (*type:* `str`): ID name from the Gazebo model 
        to be imported
        
        > *Returns*
        
        `True` if Gazebo model could be included in the assets list.
        """        
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(gazebo_model_name)
        
        if hasattr(sdf, 'lights') and sdf.lights is not None:
            return self.add_lights_from_gazebo_model(gazebo_model_name)            
        else:                
            try:
                model = SimulationModel.from_gazebo_model(gazebo_model_name)
            except ValueError as ex:
                self._logger.error('Error loading Gazebo model <{}>'.format(gazebo_model_name))
                return False

            if model is None:
                self._logger.error('Gazebo model with name <{}> could not be found'.format(
                    gazebo_model_name))
                return False
            self.add_asset(model)
        return True

    def is_asset(self, name):
        """Return `True` if the model identified by the string `name`
        is part of the list of assets.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Name of the model
        """
        return name in self._assets

    def add_asset(self, model, name=None):
        """Add a new model asset that can be used by the engines and 
        added to the generated world.
        
        > *Input arguments*
        
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model        
        """        
        if isinstance(model, dict):
            # This model is going to be generated using the factory
            # functions in the creators module
            assert 'type' in model and 'args' in model, 'Invalid input for model creator'
            assert model['type'] in ['box', 'sphere', 'cylinder', 'mesh'], \
                'Model creator must be configured for single model generation'
            assert name is not None, 'No name identifier given for model factory input'
            self._assets[name] = model
            PCG_ROOT_LOGGER.info(
                'New model factory function was created, name={}, model={}'.format(name, model))
        elif callable(model):
            # This model will be generated using a lambda function or a function handle
            assert name is not None, 'No name identifier given for model lambda function'        
            self._assets[name] = model
        elif isinstance(model, SimulationModel):
            if model.name in self._assets:
                PCG_ROOT_LOGGER.warning('Model {} is already an asset'.format(model.name))
            else:            
                self._assets[model.name] = model            
                PCG_ROOT_LOGGER.info('New model asset added, name={}'.format(model.name))
        else:
            msg = 'Invalid simulation asset'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)        

    def add_lights_from_gazebo_model(self, model_name):
        """Add light models to the generated world from a Gazebo model.
        
        > *Input arguments*
        
        * `model_name` (*type:* `str`): Name of the Gazebo model
        
        > *Returns*
        
        `True` if the lights could be parsed and added to the world.
        """
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(model_name)
        if sdf is None:
            PCG_ROOT_LOGGER.error('Model {} is not a Gazebo model'.format(model_name))
            return False
        if sdf.lights is not None:
            PCG_ROOT_LOGGER.info(
                'Input Gazebo model {} contains lights, adding to world'.format(
                    model_name))
            for light in sdf.lights:
                PCG_ROOT_LOGGER.info('Add light {}, type={}'.format(
                    light.name, light.type))
                self.world.add_light(light.name, Light.from_sdf(light))
            return True
        else:
            PCG_ROOT_LOGGER.error('Input Gazebo model contains no lights, model_name={}'.format(model_name))
            return False

    
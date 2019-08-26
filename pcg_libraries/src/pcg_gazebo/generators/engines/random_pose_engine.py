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
from .engine import Engine
import numpy as np
import random
from copy import deepcopy
from shapely.ops import cascaded_union
from shapely.affinity import rotate, translate
from ..occupancy import get_occupied_area


class RandomPoseEngine(Engine):
    """Placement engine that generates a random pose for its model 
    assets respecting input local constraints, if any is provided, 
    such as workspace constraint. This engine performs also a
    collision check with all models already placed in the scene (except 
    for models flagged as ground plane) to ensure no models are overlapping 
    each other.    
    
    > *Input arguments*
    
    * `assets_manager` (*type:* `pcg_gazebo.generators.AssetsManager`)
    * `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`): 
    Handle to a function or a lambda function that returns a 
    `pcg_gazebo.constraints.Constraint` associated with a tag name.
    * `models` (*type:* `list`, *default:* `None`): List of models names as `str`
    relative to the models that the engine will have as assets.
    * `constraints` (*type:* `list`, *default:* `None`): List of local constraint
    configurations that will be applied on to the engine's model assets.        
    * `max_num` (*type:* `dict`, *default:* `None`): Maximum number of instances 
    of the model assets, the key being the model asset's name, and the value 
    the maximum number.
    * `no_collision` (*type:* `bool`, *default:* `True`): If `True`, the model
    instances are only added to the world if there are no collisions with the
    already existing models (except for models flagged as ground plane).
    * `max_area` (*type:* `float`, *default:* `1`): Percentage of the allowed
    area to fill with the models.
    * `model_picker` (*type:* `str`, *default:* `random`): Strategy for picking
    a model from the list of assets for the next placement in the world. Options
    are `random` (selecting a random model from the list of assets) or `area` 
    (selecting the models for the biggest to the smallest).
    * `policies` (*type:* `dict`, *default:* `None`): The rules for model
    generation associated with each degree of freedom.

    ```yaml
    policies:
        - models:
         - model_1
         - model_2
         - model_3     
         config:
         - dofs:
           - x
           - y
           policy:
             name: workspace
             args: area_1     # For more information on workspaces, check the class definition for `pcg_gazebo.constraints.WorkspaceConstraint`
         - dofs:          
           - z
           - roll
           - pitch
           policy: 
             name: value
             args: 0
         - dofs:
           - yaw
           policy:
             name: uniform
             args:
                min: -3.141592653589793
                max: 3.141592653589793
    ```
    """

    _LABEL = 'random_pose'

    _MODEL_PICKER = ['random', 'size']

    def __init__(self, assets_manager=None, callback_fcn_get_constraint=None,
        is_ground_plane=False, models=None, max_num=None, no_collision=True, 
        max_area=1, constraints=None, policies=None, model_picker='random',
        collision_checker=None):        
        Engine.__init__(
            self, 
            assets_manager=assets_manager,
            callback_fcn_get_constraint=callback_fcn_get_constraint,
            models=models, 
            constraints=constraints,
            collision_checker=collision_checker)
            
        assert model_picker in self._MODEL_PICKER, 'Model picking method options are {}'.format(
            self._MODEL_PICKER)
        assert policies is not None, 'DoF configuration was not defined'

        self._no_collision = no_collision
        self._max_num = dict()
        self._workspace = None
        self._model_picker = model_picker
        self._cached_footprints = dict()

        # Compute the footprint area of each object
        self._volumes = dict()
        
        for name in self._models:
            self._max_num[name] = -1

            if self._assets_manager.is_model_group(name) or \
                self._assets_manager.is_gazebo_model(name) or \
                    self._assets_manager.is_model(name):
                model = self._assets_manager.get(name)
                self._volumes[name] = 0
                for mesh in model.get_meshes():
                    self._volumes[name] += mesh.volume
            else:
                self._volumes[name] = -1

        self._policies = policies

        self._has_repeated_models()
        for policy in self._policies:
            self._has_repeated_dofs(policy)

        if max_num is not None:
            assert isinstance(max_num, dict), \
                'Max. number of models input must be a dictionary'
            for tag in max_num:
                assert tag in self._models, \
                    'Invalid input model in max_num input, tag={}'.format(tag)
                assert max_num[tag] > 0, \
                    'Max. number of models must be either greater than zero' \
                    ', received={}'.format(
                        max_num[tag])
                self._max_num[tag] = max_num[tag]
        if constraints is not None:
            assert isinstance(constraints, list), \
                'Constraints input must be provided as a list'
        self._counters = dict()

        for tag in self._models:
            self._counters[tag] = 0

        self._fixed_models_footprints = None
                
    def __str__(self):
        msg = 'Engine: {}\n'.format(self._LABEL)
        if len(self._models) > 0:
            for name in self._models:
                msg += '\tModel: {}, Max. instances: {}\n'.format(name, self._max_num[name])
        else:
            msg += '\tNo models\n'
        return msg

    def _has_repeated_dofs(self, policy):
        """Check if policies don't interfere with repeated DoFs."""
        dofs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for config in policy['config']:
            for dof in config['dofs']:
                assert dof in ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], 'Invalid DoF'
                if dof in dofs:
                    dofs.remove(dof)
                else:
                    raise ValueError('Randomization policies have repeated DoFs')
    
    def _has_repeated_models(self):
        """Check if policies have repeated model associations."""
        models = deepcopy(self._models)
        for policy in self._policies:
            for model in policy['models']:
                assert model in self._models, 'Invalid asset for random generation'
                assert model in models, 'Repeated policy for model ' + model
                models.remove(model)

    def _get_policies(self, model_name):
        """Retrieve a placement policy for a model.
        
        > *Input arguments*
        
        * `model_name` (*type:* `str`): Name of the model asset
        
        > *Returns*
        
        `dict`: Policy definition
        """
        for i in range(len(self._policies)):
            if model_name in self._policies[i]['models']:
                return self._policies[i]['config']
        return None

    def _get_random_pose(self, model_name):
        """Compute a random pose for a model respecting its placement policies 
        and workspace constraints, if provided.
        
        > *Input arguments*
        
        * `model_name` (*type:* `str`): Name of the model asset
        
        > *Returns*
        
        `list`: Pose vector
        """
        pose = [0 for _ in range(6)]
        dofs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for config in self._get_policies(model_name):
            # Apply the workspace constraint 
            if config['policy']['name'] == 'workspace':
                assert set(['x', 'y']) == set(config['dofs']) or \
                    set(['x', 'y', 'z']) == set(config['dofs']), 'For workspace policy, either x, y'\
                        ' or x, y, z components must be used'
                # Retrieve workspace instance
                if self._workspace is None:
                    self._workspace = self._get_constraint(config['policy']['args'])
                
                assert self._workspace is not None, 'Invalid workspace constraint {}'.format(
                    config['policy']['args'])

                pnt = self._workspace.get_random_position()

                if 'x' in config['dofs']:
                    pose[0] = pnt.x
                if 'y' in config['dofs']:
                    pose[1] = pnt.y
                if pnt.has_z and 'z' in config['dofs']:
                    pose[2] = pnt.z
            elif config['policy']['name'] == 'value':
                assert isinstance(config['policy']['args'], int) or isinstance(config['policy']['args'], float), \
                    'Argument for policy value must be an integer or a float'
                for i in range(len(dofs)):
                    if dofs[i] in config['dofs']:
                        pose[i] = config['policy']['args']
            elif config['policy']['name'] == 'uniform':
                min_value = 0
                max_value = 1

                if 'args' in config['policy']:
                    if 'min' in config['policy']['args']:
                        min_value = config['policy']['args']['min']
                    if 'max' in config['policy']['args']:
                        max_value = config['policy']['args']['max']

                assert min_value < max_value, \
                    'For the uniform distribution, min < max must hold'
                for i in range(len(dofs)):
                    if dofs[i] in config['dofs']:
                        pose[i] = random.uniform(
                            min_value, 
                            max_value)
            elif config['policy']['name'] == 'choice':
                assert 'args' in config['policy'], \
                    'No arguments found for <choice> policy on the '\
                        'placement of model <{}>'.format(model_name)
                assert 'values' in config['policy']['args'], \
                    'List of values must be available for <choice> ' \
                        'policy on the placement of model <{}>'.format(model_name)
                assert isinstance(config['policy']['args']['values'], list), \
                    'Input <values> for <choice> policy must be a list of scalars'                
                for i in range(len(dofs)):
                    if dofs[i] in config['dofs']:
                        value = random.choice(config['policy']['args']['values'])                        
                        pose[i] = float(value)
        
        return pose

    def reset_counter(self):
        """Reset all model counters."""
        for tag in self._counters:
            self._counters[tag] = 0

    def increase_counter(self, name):
        """Increase the counter for a model.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Model name
        """
        self._counters[name] += 1

    def get_num_models(self, name):
        """Return the current value for the model counter.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Model name
        
        > *Returns*
        
        `int`: Number of models
        """
        return self._counters[name]

    def get_max_num_models(self, name):
        """Return the defined maximum number of instances for a model.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): Model name
        
        > *Returns*
        
        `int`: Maximum number of instances
        """
        return self._max_num[name]

    def choose_model(self, models=None):
        """Select the next model instance to be placed in the world.
        This method is affected by the constructor input `model_picker`.
        In case the `model_picker` option was set as `random`, a random
        model will be chosen from the assets available. If it is `area`,
        the models will be ordered by footprint size and the models are
        chosen by an descending footprint size.
                
        > *Returns*
        
        `pcg_gazebo.simulation.SimulationModel`: Chosen model
        """
        if models is None:
            models = deepcopy(self._models)
        model = None

        if self._model_picker == 'random':
            model = random.choice(models)
            if self.get_max_num_models(model) == self.get_num_models(model):
                models.remove(model)
                if len(models) == 0:
                    return None
                while len(models) > 0:
                    model = random.choice(models)

                    if self.get_max_num_models(model) == self.get_num_models(model):
                        models.remove(model)
                    else:
                        break

                if self.get_max_num_models(model) == self.get_num_models(model):
                    return None
        elif self._model_picker == 'size':
            if len(models) == 1:
                if self.get_max_num_models(models[0]) == self.get_num_models(models[0]):
                    return None
                else:
                    return models[0]

            volumes = list(self._volumes.values())
            max_volume = np.max(volumes)
            
            while model is None:
                for tag in self._volumes:
                    if self._volumes[tag] == max_volume:
                        if self.get_max_num_models(tag) == self.get_num_models(tag):
                            volumes.remove(max_volume)
                            if len(volumes) == 0:
                                return None
                            max_volume = np.max(volumes)
                        else:
                            model = tag
        return model

    def is_model_in_workspace(self, model):
        """Verify if the model is in the allowed workspace
        
        > *Input arguments*
        
        * `footprint` (*type:* `dict` or `shapely.geometries.Polygon`): A `shapely` 
        polygon or a dictionary with the values being the footprints for different
        submodels.
        
        > *Returns*
        
        `bool`: `True` if the polygon is entirely contained inside the workspace
        """
        meshes = model.get_meshes()                
        for mesh in model.get_meshes():
            if not self._workspace.contains_mesh(mesh):
                return False
        return True
        
    def get_list_of_footprint_polygons(self, footprint):
        """Return the list of polygons contained in the `footprint` input.
                
        > *Input arguments*
        
        * `footprint` (*type:* `dict` or `shapely.geometries.Polygon`): A `shapely` 
        polygon or a dictionary with the values being the footprints for different
        submodels.
        
        > *Returns*
        
        List of `shapely.geometry.Polygon`: List of footprint polygons
        """
        polys = list()
        if isinstance(footprint, dict):
            for tag in footprint:
                polys = polys + self.get_list_of_footprint_polygons(footprint[tag])
        else:
            polys = [footprint]
        return polys

    def has_collision(self, model):
        """Run the collision checker of the input `model`
        against the current scene of the simulation.
        
        > *Input arguments*
        
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): 

        
        > *Returns*
        
        `bool`: `True`, if any collision is detected
        """
        return self._collision_checker.check_collision_with_current_scene(model)

    def run(self):
        """Run the placement engine and generate a list of models placed
        according to the input policies and respecting spatial constraints.
        
        > *Returns*
        
        List of `pcg_gazebo.simulation.SimulationModel`
        """
        if len(self.models) == 0:
            self._logger.error('No model was provided for fixed pose engine')
            return None
        models = list()

        # Reset model counter
        self.reset_counter()

        # Reset the collision checker scenario to fixed model scenario
        self._collision_checker.reset_to_fixed_model_scenario()

        collision_counter = 0
        model_reset_counter = 0
        max_collision_per_iter = 50
        max_num_model_resets = 10
        
        while True:
            if collision_counter == max_collision_per_iter:
                if max_num_model_resets == model_reset_counter:
                    self._logger.info('Unable to fit all models in the provided workspace')
                    break
                self._logger.info('Reset models list, max. number of collisions was reached')
                self.reset_counter()
                self._collision_checker.reset_to_fixed_model_scenario()
                models = list()
                collision_counter = 0
                model_reset_counter += 1

            model_name = self.choose_model()
            if model_name is None:
                self._logger.info('Maximum number of models reached')
                break
            self._logger.info('Chosen model: {}'.format(model_name))
            
            # Retrieve model
            model = self._get_model(model_name)
            if model is None:
                self._logger.error('Cannot spawn model <{}>'.format(self._models[0]))
                return None
            else:
                pose = self._get_random_pose(model_name)
                self._logger.info('Generated random pose: {}'.format(pose))

                model.pose = pose
                
                while not self.is_model_in_workspace(model):
                    self._logger.info('Model outside of the workspace or in collision with other objects!')
                    pose = self._get_random_pose(model_name)
                    self._logger.info('\t Generated random pose: {}'.format(pose))
                    model.pose = pose
                    
                # Enforce positioning constraints
                model = self.apply_local_constraints(model)
                if self._no_collision:
                    if self.has_collision(model):
                        self._logger.info(
                            'Collision for model {} detected, increasing collision'
                            ' counter and choosing new pose, collision counter={}'.format(model_name, collision_counter + 1))
                        collision_counter += 1
                        continue
                    else:
                        collision_counter = 0

            models.append(model)
            if not self._assets_manager.is_light(model_name):
                self._collision_checker.add_model(model)

            # Increase the counter for this chosen model
            self.increase_counter(model_name)
                        
        self._logger.info('# models:')
        for tag in self._counters:
            self._logger.info('\t - {} = {}'.format(tag, self._counters[tag]))

        # Add the models to the collision checker's fixed models list
        for model in models:
            self._collision_checker.add_fixed_model(model)
            
        return models
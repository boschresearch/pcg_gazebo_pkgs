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
from ..log import PCG_ROOT_LOGGER


class ModelGroup(object):
    def __init__(self, name='group'):
        self._logger = PCG_ROOT_LOGGER
        
        self._name = name
        self._assets = dict()
        self._engines = list()
        self._constraints = dict()

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
        input_args['callback_fcn_get_model'] = self.get_asset
        input_args['callback_fcn_get_constraint'] = self.get_constraint
        self._engines.append(create_engine(engine_name, **input_args))
        self._logger.info('New model creator engine added, type={}'.format(engine_name))
        self._logger.info(self._engines[-1].__str__())

    
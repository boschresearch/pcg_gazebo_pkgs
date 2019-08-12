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
from .constraints import create_constraint
from ._collection_manager import _CollectionManager
from ..log import PCG_ROOT_LOGGER


class ConstraintsManager(_CollectionManager):
    def __init__(self):
        super(ConstraintsManager, self).__init__()

    @staticmethod
    def get_instance():
        if ConstraintsManager._INSTANCE is None:
            ConstraintsManager._INSTANCE = ConstraintsManager()
        return ConstraintsManager._INSTANCE

    def add(self, name, type, **kwargs):
        """Add a new positioning constraint class to the internal 
        constraints list.
        
        > *Input arguments*
        
        * `name` (*type:* `str`): ID name for the constraint class instance
        * `type` (*type:* `str`): Name of the constraints class to be created
        * `kwargs` (*type:* `dict`): Input arguments for the constraint class 
        to be created        
        """
        if self.has_element(name):
            PCG_ROOT_LOGGER.error('Constraint with name <{}> already exists'.format(name))
            return False
        self._collection[name] = create_constraint(type, **kwargs)
        return True

    def from_dict(self, config):
        assert isinstance(config, dict) or isinstance(config, list), \
            'Input must be either a dictionary or a list of dictionaries'

        if isinstance(config, list):
            for elem in config:
                self.from_dict(elem)
        else:
            if not self.add(**config):
                PCG_ROOT_LOGGER.error('Failed to parse constraint configuration={}'.format(config))
            
    def from_yaml(self, filename):
        pass
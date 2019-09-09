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
"""Simulation interface module, with abstraction classes for all relevant
entities that form a simulation in Gazebo.
"""
import sys
import os
import inspect

from . import properties
from . import physics
from ..log import PCG_ROOT_LOGGER
from .box import Box
from .cylinder import Cylinder
from .joint import Joint
from .light import Light
from .model_group import ModelGroup
from .model import SimulationModel
from .link import Link
from .plane import Plane
from .polyline import Polyline
from .sphere import Sphere
from .world import World


GAZEBO_MODELS = dict()


def create_object(tag, **kwargs):
    """Factory method for `Link` subclasses.
    
    > *Input arguments*
    
    * `tag` (*type:* `str`): Name identifier of the object class
    * `kwargs` (*type:* `dict`): Input arguments for the object class
    
    > *Returns*
    
    `Link`: Subclass instance.
    """
    for obj in Link.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Link):
                if tag == obj.__name__.lower():
                    return obj(**kwargs)
    PCG_ROOT_LOGGER.error('Object {} does not exist'.format(tag))
    return None


def get_gazebo_model_folders(dir_path):
    """Return the paths to all Gazebo model folders under the 
    directory `dir_path`.
    
    > *Input arguments*
    
    * `dir_path` (*type:* `str`): Path to the search directory.
    
    > *Returns*
    
    `dict`: Gazebo model paths ordered according to the 
    Gazebo model names.
    """
    import os
    assert os.path.isdir(dir_path), \
        'Invalid directory path, path={}'.format(dir_path)

    models_paths = dict()
    for item in os.listdir(dir_path):
        if os.path.isdir(os.path.join(dir_path, item)):
            subfolder_items = os.listdir(os.path.join(dir_path, item))
            has_config = False
            has_sdf = False
            sdf_files = list()

            for subitem in os.listdir(os.path.join(dir_path, item)):
                if os.path.isfile(os.path.join(dir_path, item, subitem)):
                    if '.config' in subitem:
                        has_config = True
                    if '.sdf' in subitem:
                        has_sdf = True
                        sdf_files.append(subitem)
            
            if has_config and has_sdf:
                models_paths[item] = dict(
                    path=os.path.join(dir_path, item), 
                    sdf=sdf_files)
            else:
                models_paths.update(get_gazebo_model_folders(os.path.join(dir_path, item)))        
    return models_paths

def load_gazebo_models():
    """Search for Gazebo models in the local `.gazebo/models` folder
    and in the ROS paths.

    > *Returns*

    `dict`: Information of all Gazebo models found
    """
    import rospkg
    import os

    finder = rospkg.RosPack()

    global GAZEBO_MODELS
    GAZEBO_MODELS = dict()

    # Load all models from catkin packages
    for ros_pkg in finder.list():        
        ros_path = finder.get_path(ros_pkg)
        for folder in os.listdir(ros_path):
            if not os.path.isdir(os.path.join(ros_path, folder)):
                continue
            models = get_gazebo_model_folders(os.path.join(ros_path, folder))
            for tag in models:
                models[tag]['ros_pkg'] = ros_pkg
            GAZEBO_MODELS.update(models)

    # Load all models from ~/.gazebo/models
    home_folder = os.path.expanduser('~')
    gazebo_folder = os.path.join(home_folder, '.gazebo', 'models')
    if os.path.isdir(gazebo_folder):
        GAZEBO_MODELS.update(get_gazebo_model_folders(gazebo_folder))

    folders = os.listdir('/usr/share')
    gazebo_folder = None
    for folder in os.listdir('/usr/share'):
        if 'gazebo-' in folder:
            gazebo_folder = os.path.join('/usr', 'share', folder, 'models')
            break
            
    if gazebo_folder is not None:
        if os.path.isdir(gazebo_folder):
            GAZEBO_MODELS.update(get_gazebo_model_folders(gazebo_folder))
    return GAZEBO_MODELS


def get_gazebo_models():
    """Return the information of all Gazebo models found in the
    local `.gazebo/models` folder and in the catkin workspace as
    a dictionary.
    """
    load_gazebo_models()
    return GAZEBO_MODELS
    

def get_gazebo_model_names():    
    """Return the names of all Gazebo models that can be found 
    is the local `.gazebo/models` folders and catkin workspace.
    """
    load_gazebo_models()
    return GAZEBO_MODELS.keys()


def get_gazebo_model_ros_pkg(name):
    """Return name of the ROS package where the Gazebo model is
    located, None if it was found in .gazebo/models.
    """
    if name not in GAZEBO_MODELS:
        # Try reloading the models
        load_gazebo_models()
    if not is_gazebo_model(name):
        raise ValueError('{} is not a Gazebo model'.format(name))
    if 'ros_pkg' in GAZEBO_MODELS[name]:
        return GAZEBO_MODELS[name]['ros_pkg']
    else:
        return None


def is_gazebo_model(name):
    """Test if a model with the identifier `name` is a Gazebo 
    model that is found in the resources path.
    
    > *Input arguments*
    
    * `name` (*type:* `str`): Name identifier of the model
    
    > *Returns*
    
    `True` if `name` refers to a Gazebo model.
    """
    if name not in GAZEBO_MODELS:
        # Try reloading the models
        load_gazebo_models()
    return name in GAZEBO_MODELS

def get_gazebo_model_sdf_filenames(model_name):
    if model_name not in GAZEBO_MODELS:
        # Try reloading the models
        load_gazebo_models()
        if model_name not in GAZEBO_MODELS:
            PCG_ROOT_LOGGER.error('Model {} could not be found'.format(model_name))
            return None
    return GAZEBO_MODELS[model_name]['sdf']


def get_gazebo_model_path(model_name):
    """Return the path of the Gazebo model.
    
    > *Input arguments*
    
    * `model_name` (*type:* `str`): Name of the Gazebo model
    
    > *Returns*
    
    `str`: Path of the Gazebo model folder
    """
    if model_name not in GAZEBO_MODELS:
        # Try reloading the models
        load_gazebo_models()
        if model_name not in GAZEBO_MODELS:
            PCG_ROOT_LOGGER.error('Model {} could not be found'.format(model_name))
            return None
    return GAZEBO_MODELS[model_name]['path']


def get_gazebo_model_sdf(model_name, sdf_file='model.sdf'):
    """Parse the Gazebo model's SDF file into a `pcg_gazebo` 
    SDF instance.
    
    > *Input arguments*
    
    * `model_name` (*type:* `str`): Name of the Gazebo model.
    * `sdf_file` (*type:* `str`, *default:* `model.sdf`): Name 
    of the SDF file to be parsed.
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` instance as an SDF element.
    """
    import os
    from ..parsers import parse_sdf

    if model_name not in GAZEBO_MODELS:
        # Try reloading the models        
        
        load_gazebo_models()
        if model_name not in GAZEBO_MODELS:
            PCG_ROOT_LOGGER.error('Model {} could not be found'.format(model_name))
            return None
    
    if sdf_file not in GAZEBO_MODELS[model_name]['sdf']:
        PCG_ROOT_LOGGER.error('SDF file {} not found for model {}, options are={}'.format(
            sdf_file, model_name, GAZEBO_MODELS[model_name]['sdf']))
        return None

    return parse_sdf(os.path.join(GAZEBO_MODELS[model_name]['path'], sdf_file))

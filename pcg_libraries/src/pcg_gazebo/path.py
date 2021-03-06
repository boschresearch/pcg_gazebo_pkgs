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
import os
import re
import sys
import rospkg
from .log import PCG_ROOT_LOGGER
from .utils import PCG_RESOURCES_ROOT_DIR

class Path(object):
    def __init__(self, uri):
        if sys.version_info[0] == 2:
            assert isinstance(uri, str) or isinstance(uri, unicode), \
                'Input URI must be a string or unicode'
        else:
            assert isinstance(uri, str), 'Input URI must be a string'

        self._original_uri = uri
        self._gazebo_model = None
        self._ros_pkg = None
        self._absolute_uri = self.resolve_uri(uri)
        if self._absolute_uri is None:
            msg = 'URI could not be resolved, uri={}'.format(uri)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(uri)

        PCG_ROOT_LOGGER.info('URI {} resolved={}'.format(uri, self.absolute_uri))

    @property
    def original_uri(self):
        return self._original_uri

    @property
    def absolute_uri(self):
        return self._absolute_uri

    @property
    def ros_package(self):
        if self._ros_pkg is None:
            self._resolve_ros_package()
        return self._ros_pkg

    @property
    def gazebo_model(self):
        if self._gazebo_model is None:
            self._resolve_gazebo_model()
        return self._gazebo_model

    @property
    def file_uri(self):
        return 'file://{}'.format(self._absolute_uri)

    @property
    def package_uri(self):
        if self._ros_pkg is None:
            return None
        relative_path = self._absolute_uri.replace(
            rospkg.RosPack().get_path(self._ros_pkg), '')
        prefix = 'package://{}'.format(self._ros_pkg)
        if relative_path[0] != '/':
            prefix += '/'
        return prefix + relative_path

    @property
    def model_uri(self):
        from .simulation import get_gazebo_model_path
        if self._gazebo_model is None:
            return None
        model_path = get_gazebo_model_path(self._gazebo_model)
        relative_path = self._absolute_uri.replace(model_path, '')
        prefix = 'model://{}'.format(self._gazebo_model)
        if relative_path[0] != '/':
            prefix += '/'
        return prefix + relative_path

    @property
    def ros_package_uri(self):
        self.resolve_uri(self._absolute_uri)
        if self._ros_pkg is None:
            return None
        relative_path = self._absolute_uri.replace(
            rospkg.RosPack().get_path(self._ros_pkg), 
            '')
        prefix = '$(find {})'.format(self._ros_pkg)
        if relative_path[0] != '/':
            prefix += '/'
        return prefix + relative_path

    def _get_ros_package_name(self, filename):
        finder = rospkg.RosPack()
        if os.path.isfile(filename):
            for pkg_name in finder.list():
                if finder.get_path(pkg_name) in filename:
                    return pkg_name 
        return None            

    def resolve_uri(self, uri):
        from .simulation import get_gazebo_model_path, \
            get_gazebo_model_ros_pkg
        if os.path.isfile(uri):
            return uri
        elif 'file://' in uri:
            filename = uri.replace('file://', '')
            if os.path.isfile(filename):
                self._ros_pkg = self._get_ros_package_name(filename)
                return filename
            else:
                msg = 'File {} does not exist'.format(filename)
                PCG_ROOT_LOGGER.error(msg)
                raise FileNotFoundError(msg)
        elif 'model://' in uri:
            self._gazebo_model = uri.replace('model://', '').split('/')[0]
            model_path = get_gazebo_model_path(self._gazebo_model)
            if model_path is None:
                msg = 'URI for Gazebo model is invalid, uri={}'.format(uri)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)
            self._ros_pkg = get_gazebo_model_ros_pkg(self._gazebo_model)
            filename = uri.replace('model://{}'.format(self._gazebo_model), model_path)
            self._ros_pkg = self._get_ros_package_name(filename)
            return filename
        elif 'package://' in uri:
            result = re.findall('package://\w+/', uri)
            if len(result) != 1:
                msg = 'Invalid package path for provided mesh uri {}'.format(uri)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)
            self._ros_pkg = result[0].replace('package://', '').replace('/', '')
            if self._ros_pkg not in rospkg.RosPack().list():
                msg = 'Package {} was not found, uri={}'.format(self._ros_pkg, uri)
                PCG_ROOT_LOGGER.error(msg)
                raise rospkg.ResourceNotFound(msg)
            pkg_path = rospkg.RosPack().get_path(self._ros_pkg)
            return uri.replace(result[0], pkg_path + '/')
        elif '$(find' in uri:
            uri_temp = uri.replace('$', '')
            result = re.findall('(find \w+)', uri_temp)
            if len(result) == 0:
                msg = 'Invalid package path for provided mesh uri {}'.format(uri)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

            self._ros_pkg = result[0].split()[1]

            if self._ros_pkg not in rospkg.RosPack().list():
                msg = 'Package {} was not found, uri={}'.format(self._ros_pkg, uri)
                PCG_ROOT_LOGGER.error(msg)
                raise rospkg.ResourceNotFound(msg)
            pkg_path = rospkg.RosPack().get_path(self._ros_pkg)
            return uri_temp.replace('(find {})'.format(self._ros_pkg), pkg_path)            
        else:
            return None

    def _resolve_gazebo_model(self):
        from .simulation import get_gazebo_model_names, get_gazebo_model_path, \
            load_gazebo_models

        load_gazebo_models()
        for name in get_gazebo_model_names():
            gazebo_path = get_gazebo_model_path(name)
            if gazebo_path in os.path.dirname(self.absolute_uri):
                self._gazebo_model = name
                return 

    def _resolve_ros_package(self):
        for ros_pkg in rospkg.RosPack().list():
            pkg_path = rospkg.RosPack().get_path(ros_pkg)
            if pkg_path in os.path.dirname(self.absolute_uri):
                self._ros_pkg = ros_pkg
                return
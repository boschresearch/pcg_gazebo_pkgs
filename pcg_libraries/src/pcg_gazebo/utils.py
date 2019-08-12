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
import random
import string
import os
import yaml

class _PCGYAMLLoader(yaml.SafeLoader, object):
    # MIT License
    #
    # Copyright (c) 2018 Josh Bode
    #
    # Permission is hereby granted, free of charge, to any person 
    # obtaining a copy of this software and associated documentation 
    # files (the "Software"), to deal in the Software without restriction, 
    # including without limitation the rights to use, copy, modify, merge, 
    # publish, distribute, sublicense, and/or sell copies of the Software, and 
    # to permit persons to whom the Software is furnished to do so, subject 
    # to the following conditions:
    #
    # The above copyright notice and this permission notice shall be included 
    # in all copies or substantial portions of the Software.

    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
    # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    # THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    # FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
    # IN THE SOFTWARE.

    """YAML Loader with `!include` constructor."""

    def __init__(self, stream):
        """Initialise Loader."""

        try:
            self._root = os.path.split(stream.name)[0]
        except AttributeError:
            self._root = os.path.curdir

        super(_PCGYAMLLoader, self).__init__(stream)


def yaml_include_constructor(loader, node):
    # MIT License
    #
    # Copyright (c) 2018 Josh Bode
    #
    # Permission is hereby granted, free of charge, to any person 
    # obtaining a copy of this software and associated documentation 
    # files (the "Software"), to deal in the Software without restriction, 
    # including without limitation the rights to use, copy, modify, merge, 
    # publish, distribute, sublicense, and/or sell copies of the Software, and 
    # to permit persons to whom the Software is furnished to do so, subject 
    # to the following conditions:
    #
    # The above copyright notice and this permission notice shall be included 
    # in all copies or substantial portions of the Software.

    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
    # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    # THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    # FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
    # IN THE SOFTWARE.
    filename = os.path.abspath(os.path.join(loader._root, loader.construct_scalar(node)))
    
    with open(filename, 'r') as f:
        return yaml.load(f, _PCGYAMLLoader)


def yaml_find_ros_package(loader, node):
    import rospkg
    input_str = loader.construct_scalar(node)
    assert '/' in input_str, \
        'ROS package to be searched must be provided' \
        ' as <ros_package/<path_to_file>'
    ros_package = input_str.split('/')[0]
    
    finder = rospkg.RosPack()
    assert ros_package in finder.list(), \
        'Could not find ROS package {}'.format(ros_package)

    ros_package_path = finder.get_path(ros_package)
    filename = input_str.replace(ros_package, ros_package_path)

    assert os.path.isfile(filename), 'Invalid filename={}'.format(filename)

    with open(filename, 'r') as f:
        return yaml.load(f, _PCGYAMLLoader)


yaml.add_constructor('!include', yaml_include_constructor, _PCGYAMLLoader)
yaml.add_constructor('!find', yaml_find_ros_package, _PCGYAMLLoader)


def load_yaml(input_yaml):
    if os.path.isfile(input_yaml):
        extension = os.path.splitext(input_yaml)[1].lstrip('.')
        assert extension in ['yaml', 'yml'], \
            'Invalid YAML file extension, expected=yaml or yml, received={}'.format(
                extension)            
        with open(input_yaml, 'r') as f:
            data = yaml.load(f, _PCGYAMLLoader)
        return data
    elif isinstance(input_yaml, str):
        return yaml.load(input_yaml, _PCGYAMLLoader)

def generate_random_string(size=3):
    return ''.join(random.choice(string.ascii_letters) for i in range(size))




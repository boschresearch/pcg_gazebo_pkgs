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
import sys
import re
import subprocess
import rospkg
import subprocess
from jinja2 import FileSystemLoader, Environment, \
    BaseLoader, TemplateNotFound
from ..log import PCG_ROOT_LOGGER


class _AbsFileSystemLoader(BaseLoader):
    def __init__(self, path):
        self.path = path

    def get_source(self, environment, template):        
        if os.path.isfile(template):
            path = template
        else:
            path = os.path.join(self.path, template)
            if not os.path.isfile(path):
                raise TemplateNotFound(template)
        mtime = os.path.getmtime(path)
        with open(path) as f:    
            source = f.read()
            if not isinstance(source, str):
                source = source.decode('utf-8')
        return source, path, lambda: mtime == os.path.getmtime(path)


def _find_ros_package(pkg_name):
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound as ex:
        print('Error finding package {}, message={}'.format(pkg_name, ex))
        sys.exit(-1)    
    return pkg_path


def _pretty_print_xml(xml):
    import lxml.etree as etree
    root = etree.fromstring(xml)
    return etree.tostring(root, pretty_print=True, encoding='utf-8').decode('utf-8')


def _parse_package_paths(xml):
    # Finding patterns package://
    result = re.findall('package://\w+/', xml)
    output_xml = xml
    for item in result:        
        pkg_name = item.replace('package://', '').replace('/', '')
        try:
            pkg_path = rospkg.RosPack().get_path(pkg_name)
        except rospkg.ResourceNotFound as ex:
            print('Error finding package {}, message={}'.format(pkg_name, ex))
            sys.exit(-1)

        output_xml = output_xml.replace(item, 'file://' + pkg_path + '/')

    # Finding patterns $(find pkg)
    result = re.findall(r'\$\(find \w+\)', output_xml)        
    for item in result:                
        pkg_name = item.split()[1].replace(')', '')
        try:
            pkg_path = rospkg.RosPack().get_path(pkg_name)
        except rospkg.ResourceNotFound as ex:
            print('Error finding package {}, message={}'.format(pkg_name, ex))
            sys.exit(-1)

        output_xml = output_xml.replace(item, 'file://' + pkg_path + '/')
    return output_xml        


def process_template(template, parameters=None, include_dir=None):    
    if isinstance(include_dir, str):
        if not os.path.isdir(include_dir):            
            PCG_ROOT_LOGGER.error('Include directory in invalid, dir={}'.format(include_dir))
            return None
    else:
        include_dir = '.'

    if not isinstance(parameters, dict):
        parameters = dict()
        PCG_ROOT_LOGGER.warning(
            'Input parameters to be replaced in the template'
            ' must be provided as a dictionary, received {}'
            ' instead'.format(type(parameters)))
        
    if not isinstance(template, str):
        PCG_ROOT_LOGGER.error(
            'The template input must a string, either with the'
            ' template text content or filename to the template')
        return None
    
    PCG_ROOT_LOGGER.info('Input template: {}'.format(template))
    if os.path.isfile(template):
        PCG_ROOT_LOGGER.info('Input template is a file, {}'.format(template))
        templates_dir = os.path.dirname(template)
        base_loader = FileSystemLoader(templates_dir)
    else:
        base_loader = FileSystemLoader('.')
    includes_loader = _AbsFileSystemLoader(include_dir)

    base_env = Environment(loader=base_loader)
    # Add Jinja function similar to $(find <package>) in XACRO
    base_env.filters['find_ros_package'] = _find_ros_package

    if os.path.isfile(template):
        PCG_ROOT_LOGGER.info(
            'Retrieving template to be rendered from file {}'.format(template))
        model_template = base_env.get_template(os.path.basename(template))
    else:
        model_template = base_env.from_string(template)
    model_template.environment.loader = includes_loader

    output_xml = _pretty_print_xml(model_template.render(**parameters))
    output_xml = _parse_package_paths(output_xml)    

    return output_xml
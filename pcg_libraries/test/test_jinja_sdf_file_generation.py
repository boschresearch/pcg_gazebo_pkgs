#!/usr/bin/env python
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
# TODO: Replace yasha by jinja for generation of the SDF files
import roslib
import rospkg

PKG = 'pcg_libraries'
roslib.load_manifest(PKG)

import os
import sys
import unittest
import subprocess
from jinja2 import FileSystemLoader, Environment, \
    BaseLoader, TemplateNotFound
from pcg_gazebo.parsers import parse_sdf

CUR_DIR = os.path.join(rospkg.RosPack().get_path(PKG), 'test')

class AbsFileSystemLoader(BaseLoader):
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
            source = f.read().decode('utf-8')
        return source, path, lambda: mtime == os.path.getmtime(path)


def find_ros_package(pkg_name):
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound as ex:
        print('Error finding package {}, message={}'.format(pkg_name, ex))
        sys.exit(-1)    
    return pkg_path


def pretty_print_xml(xml):
    proc = subprocess.Popen(
        ['xmllint', '--format', '/dev/stdin'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        )
    (output, error_output) = proc.communicate(xml)
    return output

def generate_sdf(template, output_filename, vars):       
    base_loader = FileSystemLoader(os.path.join(CUR_DIR, 'jinja_sdf'))
    includes_loader = AbsFileSystemLoader(os.path.join(CUR_DIR, '..', 'sdf'))

    base_env = Environment(loader=base_loader)
    # Add Jinja function similar to $(find <package>) in XACRO
    base_env.filters['find_ros_package'] = find_ros_package

    model_template = base_env.get_template(template)
    model_template.environment.loader = includes_loader

    params = dict()
    for tag in vars:
        params[tag] = vars[tag]

    output_xml = pretty_print_xml(model_template.render(**params))
    with open(os.path.join('/tmp', output_filename), 'w+') as f:
        f.write(output_xml)

class TestJinjaSDFFileGeneration(unittest.TestCase):
    def test_generate_inertia(self):

        INPUT_PARAMS = dict(
            inertia_solid_sphere=dict(mass=10, radius=2),
            hollow_sphere_inertia=dict(mass=3, radius=2),
            ellipsoid_inertia=dict(mass=10, axis_length_x=2, axis_length_y=3, axis_length_z=4),
            cuboid_inertia=dict(mass=12, length_x=2, length_y=4, length_z=6),
            solid_cylinder_inertia_axis_x=dict(mass=12, radius=10, length=2),
            solid_cylinder_inertia_axis_y=dict(mass=12, radius=10, length=2),
            solid_cylinder_inertia_axis_z=dict(mass=12, radius=10, length=2)
        )

        OUTPUT_PARAMS = dict(
            inertia_solid_sphere=dict(ixx=16, ixy=0, ixz=0, iyy=16, izz=16),
            hollow_sphere_inertia=dict(ixx=8, ixy=0, ixz=0, iyy=8, izz=8),
            ellipsoid_inertia=dict(ixx=50, ixy=0, ixz=0, iyy=40, izz=26),
            cuboid_inertia=dict(ixx=52, ixy=0, ixz=0, iyy=40, izz=20),
            solid_cylinder_inertia_axis_x=dict(ixx=600, ixy=0, ixz=0, iyy=304, izz=304),
            solid_cylinder_inertia_axis_y=dict(ixx=304, ixy=0, ixz=0, iyy=600, izz=304),
            solid_cylinder_inertia_axis_z=dict(ixx=304, ixy=0, ixz=0, iyy=304, izz=600)
        )

        for test_case in INPUT_PARAMS:
            # template = os.path.join(CUR_DIR, 'jinja_sdf', '{}.jinja'.format(test_case))
            template = test_case + '.jinja'
            generate_sdf(template, '{}.sdf'.format(test_case), INPUT_PARAMS[test_case])

            filename = os.path.join('/tmp', '{}.sdf'.format(test_case))

            self.assertTrue(os.path.isfile(filename), 
                '{} file was not generated'.format(test_case + '.sdf'))

            sdf = parse_sdf(filename)

            self.assertIsNotNone(
                sdf, 
                'SDF file {} could not be parsed'.format(filename))
            self.assertEqual(
                sdf.xml_element_name, 'inertia', 
                'SDF element for file {} should be inertia')
            
            for param_name in OUTPUT_PARAMS[test_case]:
                self.assertEqual(
                    getattr(sdf, param_name).value, 
                    OUTPUT_PARAMS[test_case][param_name],
                    'Parameter {} for inertia {} is incorrect, returned={}, expected={}'.format(
                        param_name, 
                        test_case, 
                        getattr(sdf, param_name).value, 
                        OUTPUT_PARAMS[test_case][param_name]
                    ))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(
        PKG, 
        'test_jinja_sdf_file_generation',
        TestJinjaSDFFileGeneration)

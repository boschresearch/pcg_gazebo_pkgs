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
"""Parsing module to generated and convert SDF, URDF and SDF Configuration formats.

> *Sources*

* [SDF format](http://sdformat.org/)
* [URDF format specifications](https://wiki.ros.org/urdf/XML)
"""
from .jinja_template_renderer import process_template
from ..log import PCG_ROOT_LOGGER


def parse_sdf(input_xml):
    """Parse an XML file in the SDF format and generates 
    an `pcg_gazebo` SDF instance.
    
    > *Input arguments*
    
    * `input_xml` (*type:* `str`): Filename of the SDF file or
    SDF XML formatted text.
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    sdf = parse_xml(input_xml, type='sdf')
    return sdf

def parse_urdf(input_xml):
    """Parse an XML file in the URDF format and generates 
    an `pcg_gazebo` URDF instance.
    
    > *Input arguments*
    
    * `input_xml` (*type:* `str`): Filename of the URDF file or
    URDF XML formatted text.
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    urdf = parse_xml(input_xml, type='urdf')
    return urdf


def parse_sdf_config(input_xml):
    """Parse an XML file in the SDF Configuration format and generates 
    an `pcg_gazebo` SDF Configuration instance.
    
    > *Input arguments*
    
    * `input_xml` (*type:* `str`): Filename of the SDF Configuration file or
    SDF Configuration XML formatted text.
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    sdf_config = parse_xml(input_xml, type='sdf_config')
    return sdf_config
    

def parse_xml(input_xml, type='sdf'):
    """Parse an XML file into an `collections.OrderedDict`.
    
    > *Input arguments*
    
    * `input_xml` (*type:* `str`): Filename of the XML file or
    XML formatted text.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.
    
    > *Returns*
    
    `collections.OrderedDict`: Dictionary where the XML tags are the keys.
    """
    import os
    import sys

    if sys.version_info[0] == 3:
        if not isinstance(input_xml, str):        
            msg = 'Input XML must be either a filename or a string, received={}'.format(input_xml)
            PCG_ROOT_LOGGER.error(msg)
            raise Exception(msg)
    else:
        if not isinstance(input_xml, str) and not isinstance(input_xml, unicode):        
            msg = 'Input XML must be either a filename or a string, received={}'.format(input_xml)
            PCG_ROOT_LOGGER.error(msg)
            raise Exception(msg)
    
    PCG_ROOT_LOGGER.info('Parsing XML\n{}'.format(input_xml))
    if os.path.isfile(input_xml):
        filename = input_xml
        assert os.path.isfile(filename), 'File does not exist'
        output = str()
        with open(filename, 'r') as xml_file:
            for line in xml_file:
                output += line
    else:
        output = input_xml
    
    return parse_xml_str(output, type)


def parse_xml_str(xml_str, type='sdf'):
    """Parse an XML formatted string into an 
    `collections.OrderedDict`.
    
    > *Input arguments*
    
    * `input_xml` (*type:* `str`): XML formatted text.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.
    
    > *Returns*
    
    `collections.OrderedDict`: Dictionary where the XML tags are the keys.
    """
    import xmltodict
    parsed_xml = xmltodict.parse(xml_str)
    return parse_xml_dict(parsed_xml, type)


def parse_xml_dict(xml_dict, type='sdf'):
    """Converts an `collections.OrderedDict` created from a XML file
    and return an SDF, URDF or SDF Configuration `pcg_gazebo` element.
    
    > *Input arguments*
    
    * `xml_dict` (*type:* `collections.OrderedDict`): XML contents.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    from .sdf import create_sdf_element
    from .urdf import create_urdf_element
    from .sdf_config import create_sdf_config_element

    data = convert_to_dict(xml_dict)
        
    name = list(xml_dict.keys())[0]
    if type == 'sdf':
        obj = create_sdf_element(name)
    elif type == 'urdf':
        obj = create_urdf_element(name)
    elif type == 'sdf_config':
        obj = create_sdf_config_element(name)
    else:
        raise TypeError('File type {} is invalid'.format(type))
    assert obj is not None, 'Element {} does not exist'.format(name)
        
    obj.from_dict(data[name])        
    return obj

def convert_custom(xml_dict):
    import collections
    
    if isinstance(xml_dict, list):
        output = list()
        for elem in xml_dict:
            output.append(convert_custom(elem))
    else:
        tags = list(xml_dict.keys())
        output = dict()
        for tag in tags:
            if '@' in tag:
                if 'attributes' not in output:
                    output['attributes'] = dict()
                output['attributes'][tag.replace('@', '')] = convert_from_string(xml_dict[tag])                        
            elif '#' in tag:
                if 'value' not in output:
                    output['value'] = dict()
                output['value'] = convert_from_string(xml_dict[tag])
            elif isinstance(xml_dict[tag], dict) or \
                isinstance(xml_dict[tag], collections.OrderedDict):
                print(tag, xml_dict[tag])
                output[tag] = convert_custom(xml_dict[tag])
            else:
                output[tag] = convert_from_string(xml_dict[tag])
    return output 

def convert_to_dict(xml_dict):
    """Convert the `xmltodict` output into a dictionary that can be
    parsed into a `pcg_gazebo.parsers.types.XMLBase`.
    
    > *Input arguments*
    
    * `xml_dict` (*type:* `collections.OrderedDict`): XML content in 
    dictionary form.
    
    > *Returns*
    
    `dict`: Formatted XML dictionary.
    """
    import collections
    custom_elements = ['plugin']

    tags = list(xml_dict.keys())

    output = dict()

    for tag in tags:
        if tag in custom_elements:
            output[tag] = convert_custom(xml_dict[tag])
        elif '@' in tag:
            if 'attributes' not in output:
                output['attributes'] = dict()
            output['attributes'][tag.replace('@', '')] = convert_from_string(xml_dict[tag])                        
        elif '#' in tag:
            if 'value' not in output:
                output['value'] = dict()
            output['value'] = convert_from_string(xml_dict[tag])
        elif isinstance(xml_dict[tag], list):
            if tag not in output:
                output[tag] = list()            
            for elem in xml_dict[tag]:
                subelem_dict = dict()
                subelem_dict[tag] = elem
                if isinstance(subelem_dict[tag], dict) or \
                    isinstance(subelem_dict[tag], collections.OrderedDict):
                    output[tag].append(convert_to_dict(subelem_dict[tag]))
                else:
                    output[tag].append(convert_from_string(subelem_dict[tag]))             
        elif isinstance(xml_dict[tag], dict):  
            output[tag] = convert_to_dict(xml_dict[tag])                
        else:
            output[tag] = dict(value=convert_from_string(xml_dict[tag]))            

    return output


def convert_from_string(str_input_xml):
    """Convert a string into a Python data structure type.
    
    > *Input arguments*
    
    * `str_input_xml` (*type:* `str`): Input string
    
    > *Returns*
    
    `bool`, `int`, `float`, list of `float` or `str`.
    """
    import string
    if str_input_xml is None:
        return ''
    value = None

    if isinstance(str_input_xml, list):        
        value = str_input_xml
    elif str_input_xml.isnumeric():        
        value = int(str_input_xml)
    elif str_input_xml in ['true', 'false', 'True', 'False']:        
        value = True if str_input_xml in ['true', 'True'] else False
    elif ' ' in str_input_xml:                
        # Check if this a string with whitespaces
        is_numeric = True 
        for c in str_input_xml.split():
            try:
                float(c)
            except:
                is_numeric = False
                break
        
        if is_numeric:
            value = list()
            for item in str_input_xml.split():
                value.append(float(item))
    else:
        try:            
            value = float(str_input_xml)
        except ValueError:
            value = str(str_input_xml)

    return value


def sdf2urdf(sdf):
    """Recursively convert a SDF `pcg_gazebo` element and its child elements 
    into an URDF `pcg_gazebo` element.
    
    > *Input arguments*
    
    * `sdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid SDF element
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` as an URDF element.
    """
    from .sdf import create_sdf_element
    from .urdf import create_urdf_element
    from ..simulation.properties import Pose
    import numpy as np
    
    assert sdf is not None, 'Input SDF is invalid'
    SDF2URDF_OPT = dict(
        model='robot',
        link='link',
        joint='joint',
        inertial='inertial',
        inertia='inertia', 
        mass='mass',
        visual='visual',
        collision='collision',
        box='box',
        cylinder='cylinder', 
        sphere='sphere',
        mesh='mesh',
        geometry='geometry', 
        material='material',
        pose='origin',
        child='child',
        parent='parent',
        limit='limit',
        dynamics='dynamics')
        
    assert sdf._NAME in SDF2URDF_OPT.keys(), \
        'SDF element of type <{}> not available for conversion to URDF'.format(sdf._NAME)

    urdf = create_urdf_element(SDF2URDF_OPT[sdf._NAME])

    if sdf._NAME == 'model':        
        from copy import deepcopy
        model_sdf = deepcopy(sdf)
        # For the URDF file the frames must be positioned in 
        # the joint blocks instead of the links
        if model_sdf.joints:            
            # TODO Check relative link position to previous link frame
            for i in range(len(model_sdf.joints)):
                joint = model_sdf.joints[i]
                if joint.parent.value != 'world':
                    parent_link = model_sdf.get_link_by_name(joint.parent.value)
                    if parent_link is None:
                        msg = 'Parent link <{}> for joint <{}> does not exist!'.format(
                            joint.parent.value, joint.name)
                        PCG_ROOT_LOGGER.error(msg)                    
                        raise ValueError(msg)
                    if parent_link.pose is not None:
                        parent_pose = Pose(parent_link.pose.value[0:3], parent_link.pose.value[3::])
                    else:                        
                        parent_pose = Pose()
                else:
                    parent_pose = Pose()

                child_link = model_sdf.get_link_by_name(joint.child.value)   
                if child_link is None:
                    msg = 'Child link <{}> for joint <{}> does not exist!'.format(
                        joint.child.value, joint.name)
                    PCG_ROOT_LOGGER.error(msg)                    
                    raise ValueError(msg)
                
                if child_link.pose is not None:                
                    child_pose = Pose(child_link.pose.value[0:3], child_link.pose.value[3::])
                else:
                    child_pose = Pose()

                # Calculate relative pose of the joint regarding the parent's pose
                pose_diff = Pose(
                    pos=np.dot(
                        parent_pose.rotation_matrix[0:3, 0:3].T, 
                        child_pose.position - parent_pose.position),
                    quat=Pose.get_transform(parent_pose.quat, child_pose.quat)
                )

                model_sdf.joints[i].pose = pose_diff.to_sdf()

        urdf.name = model_sdf.name

        if model_sdf.links is not None:
            for link in model_sdf.links:
                urdf_link = sdf2urdf(link)
                urdf.add_link(urdf_link.name, urdf_link)

        if model_sdf.joints is not None:
            for joint in model_sdf.joints:
                urdf_joint = sdf2urdf(joint)
                urdf.add_joint(urdf_joint.name, urdf_joint)

        if model_sdf.urdf is not None:
            if model_sdf.urdf.links is not None:
                for link in model_sdf.urdf.links:
                    urdf.add_link(link.name, link)

            if model_sdf.urdf.transmissions is not None:
                for tr in model_sdf.urdf.transmissions:
                    urdf.add_transmission(tr.name, tr)

    elif sdf._NAME == 'mass':
        urdf.value = sdf.value
    elif sdf._NAME == 'inertia':
        urdf.ixx = sdf.ixx.value
        urdf.iyy = sdf.iyy.value
        urdf.izz = sdf.izz.value
        urdf.ixy = sdf.ixy.value
        urdf.ixz = sdf.ixz.value
        urdf.iyz = sdf.iyz.value
    elif sdf._NAME == 'box':
        urdf.size = sdf.size.value
    elif sdf._NAME == 'cylinder':
        urdf.length = sdf.length.value
        urdf.radius = sdf.radius.value
    elif sdf._NAME == 'sphere':
        urdf.radius = sdf.radius.value
    elif sdf._NAME == 'mesh':
        urdf.filename = sdf.uri.value
    elif sdf._NAME == 'pose':
        urdf.xyz = sdf.value[0:3]
        urdf.rpy = sdf.value[3::]
    elif sdf._NAME == 'geometry':        
        if sdf.box:            
            urdf.box = sdf2urdf(sdf.box)
        elif sdf.cylinder:
            urdf.cylinder = sdf2urdf(sdf.cylinder)
        elif sdf.sphere:
            urdf.sphere = sdf2urdf(sdf.sphere)
        elif sdf.mesh:
            urdf.mesh = sdf2urdf(sdf.mesh)
    elif sdf._NAME == 'visual':
        assert sdf.geometry is not None, \
            'Visual element has no geometry, name=' + sdf.name
        urdf.name = sdf.name
        urdf.geometry = sdf2urdf(sdf.geometry)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
        if sdf.material:
            urdf.material = sdf2urdf(sdf.material)
    elif sdf._NAME == 'collision':
        assert sdf.geometry is not None, \
            'Collision element has no geometry, name=' + sdf.name
        urdf.name = sdf.name
        urdf.geometry = sdf2urdf(sdf.geometry)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
    elif sdf._NAME == 'inertial':
        urdf.mass = sdf2urdf(sdf.mass)
        urdf.inertia = sdf2urdf(sdf.inertia)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
    elif sdf._NAME == 'joint':
        urdf.name = sdf.name
        urdf.type = sdf.type
        urdf.parent = sdf2urdf(sdf.parent)
        urdf.child = sdf2urdf(sdf.child)
        if sdf.axis:
            if sdf.axis is not None:
                urdf.axis = create_urdf_element('axis')
                urdf.axis.xyz = sdf.axis.xyz.value
            if sdf.axis.limit:
                urdf.limit = sdf2urdf(sdf.axis.limit)
            if sdf.axis.dynamics:
                urdf.dynamics = sdf2urdf(sdf.axis.dynamics)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
        
        if sdf.urdf is not None:
            if sdf.urdf.mimic is not None:
                urdf.mimic = sdf.urdf.mimic
            if sdf.urdf.safety_controller is not None:
                urdf.safety_controller = sdf.urdf.safety_controller
    elif sdf._NAME == 'parent':
        urdf.link = sdf.value
    elif sdf._NAME == 'child':
        urdf.link = sdf.value
    elif sdf._NAME == 'limit':
        if sdf.lower:
            urdf.lower = sdf.lower.value
        if sdf.upper:
            urdf.upper = sdf.upper.value
        if sdf.effort:
            urdf.effort = sdf.effort.value
        if sdf.velocity:
            urdf.velocity = sdf.velocity.value
    elif sdf._NAME == 'dynamics':
        urdf.damping = sdf.damping.value
        urdf.friction = sdf.friction.value
    elif sdf._NAME == 'material':
        if sdf.ambient:
            urdf.color = create_urdf_element('color')
            urdf.color.rgba = sdf.diffuse.value
        elif sdf.shader:
            urdf.texture.filename = sdf.shader.normal_map               
    elif sdf._NAME == 'link':
        urdf.name = sdf.name        
        if sdf.inertial:
            urdf.inertial = sdf2urdf(sdf.inertial)

        if sdf.visuals is not None:
            for visual in sdf.visuals:
                urdf_visual = sdf2urdf(visual)
                urdf.add_visual(urdf_visual.name, urdf_visual)

        if sdf.collisions is not None:
            for collision in sdf.collisions:
                urdf_collision = sdf2urdf(collision)
                urdf.add_collision(urdf_collision.name, urdf_collision)

    return urdf
        

def urdf2sdf(urdf):
    """Recursively convert an URDF `pcg_gazebo` element and its child elements 
    into a SDF `pcg_gazebo` element.
    
    > *Input arguments*
    
    * `urdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid URDF element
    
    > *Returns*
    
    `pcg_gazebo.parsers.types.XMLBase` as a SDF element.
    """
    from .sdf import create_sdf_element
    import collections

    assert urdf is not None, 'Input URDF is invalid'
    URDF2SDF_OPT = dict(
        robot='model',
        link='link',
        joint='joint',
        visual='visual',
        collision='collision',
        inertia='inertia',
        inertial='inertial',
        mass='mass',
        origin='pose',
        box='box',
        cylinder='cylinder', 
        sphere='sphere',
        mesh='mesh',
        geometry='geometry', 
        material='material',
        pose='origin',
        child='child',
        parent='parent',
        limit='limit',
        dynamics='dynamics'
    )

    assert urdf._NAME in URDF2SDF_OPT.keys(), \
        'URDF element of type <{}> not available for conversion to SDF'.format(urdf._NAME)

    sdf = create_sdf_element(URDF2SDF_OPT[urdf._NAME])

    # Parse all gazebo elements
    if urdf._NAME == 'robot':
        for gazebo in urdf.gazebos:
            if gazebo.reference is not None:
                # In case the Gazebo block is referenced to a link or joint,
                # copy child elements to the respective link or joint
                if urdf.get_link_by_name(gazebo.reference):
                    for link in urdf.links:
                        if link.name == gazebo.reference:
                            link.gazebo = gazebo
                            break
                if urdf.get_joint_by_name(gazebo.reference):
                    for joint in urdf.joint:
                        if joint.name == gazebo.reference:
                            joint.gazebo = gazebo
                            break
            else:
                # Add all model specific Gazebo elements
                for tag in gazebo.children:
                    if isinstance(gazebo.children[tag], collections.Iterable):
                        for elem in gazebo.children[tag]:
                            if elem._TYPE != 'sdf':
                                continue
                            sdf._add_child_element(elem._NAME, elem)
                    else:
                        if gazebo.children[tag]._TYPE != 'sdf':
                            continue
                        sdf._add_child_element(tag, gazebo.children[tag])

    # Set all Gazebo specific elements for this SDF element
    # in case the element can parse Gazebo elements
    if hasattr(urdf, 'gazebo'):
        if urdf.gazebo is not None:
            for tag in urdf.gazebo.children:
                if isinstance(urdf.gazebo.children[tag], collections.Iterable):
                    for elem in urdf.gazebo.children[tag]:
                        if elem._TYPE != 'sdf':
                            continue
                        sdf._add_child_element(elem._NAME, elem)
                else:
                    if urdf.gazebo.children[tag]._TYPE != 'sdf':
                        continue
                    sdf._add_child_element(tag, urdf.gazebo.children[tag])
                
    if urdf._NAME == 'mass':
        sdf.value = urdf.value
    elif urdf._NAME == 'inertia':
        sdf.ixx.value = urdf.ixx
        sdf.iyy.value = urdf.iyy
        sdf.izz.value = urdf.izz
        sdf.ixy.value = urdf.ixy
        sdf.ixz.value = urdf.ixz
        sdf.iyz.value = urdf.iyz
    elif urdf._NAME == 'box':
        sdf.size.value = urdf.size
    elif urdf._NAME == 'cylinder':
        sdf.length.value = urdf.length
        sdf.radius.value = urdf.radius
    elif urdf._NAME == 'sphere':
        sdf.radius.value = urdf.radius
    elif urdf._NAME == 'mesh':
        sdf.uri.value = urdf.filename
    elif urdf._NAME == 'origin':
        sdf.value = urdf.xyz + urdf.rpy
    elif urdf._NAME == 'geometry':        
        if urdf.box is not None:
            sdf.box = urdf2sdf(urdf.box)
        elif urdf.cylinder is not None:
            sdf.cylinder = urdf2sdf(urdf.cylinder)
        elif urdf.sphere is not None:
            sdf.sphere = urdf2sdf(urdf.sphere)
        elif urdf.mesh is not None:            
            sdf.mesh = urdf2sdf(urdf.mesh)
    elif urdf._NAME == 'visual':
        assert urdf.geometry is not None, \
            'Visual element has no geometry, name=' + urdf.name
        sdf.name = urdf.name
        sdf.geometry = urdf2sdf(urdf.geometry)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
        if urdf.material:
            sdf.material = urdf2sdf(urdf.material)
    elif urdf._NAME == 'collision':
        assert urdf.geometry is not None, \
            'Collision element has no geometry, name=' + urdf.name
        sdf.name = urdf.name
        sdf.geometry = urdf2sdf(urdf.geometry)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'inertial':
        sdf.mass = urdf2sdf(urdf.mass)
        sdf.inertia = urdf2sdf(urdf.inertia)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'joint':
        sdf.name = urdf.name        

        if urdf.type == 'continuous':
            sdf.type = 'revolute'
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.limit = create_sdf_element('limit')
            sdf.axis.limit.lower = -1e16
            sdf.axis.limit.upper = 1e16
        else:
            sdf.type = urdf.type
        sdf.parent = urdf2sdf(urdf.parent)
        sdf.child = urdf2sdf(urdf.child)
        if urdf.axis:
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.xyz = urdf.axis.xyz
        if urdf.limit and urdf.type != 'continuous':
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.limit = urdf2sdf(urdf.limit)
        if urdf.dynamics:
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.dynamics = urdf2sdf(urdf.dynamics)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'parent':
        sdf.value = urdf.link
    elif urdf._NAME == 'child':
        sdf.value = urdf.link
    elif urdf._NAME == 'limit':
        sdf.lower = urdf.lower
        sdf.upper = urdf.upper
        sdf.velocity = urdf.velocity
        sdf.effort = urdf.effort
    elif urdf._NAME == 'dynamics':
        sdf.damping = urdf.damping
        sdf.friction = urdf.friction
    elif urdf._NAME == 'material':
        if urdf.color:
            sdf.ambient = urdf.color.rgba
            sdf.diffuse = urdf.color.rgba
        elif urdf.texture:
            sdf.shader = create_sdf_element('shader')
            sdf.shader.normal_map = urdf.texture.filename
            sdf.type = 'pixel'  
    elif urdf._NAME == 'link':
        sdf.name = urdf.name
        if urdf.inertial:
            sdf.inertial = urdf2sdf(urdf.inertial)

        if urdf.gazebo is not None:
            if urdf.gazebo.selfCollide: 
                sdf.self_collide = urdf.gazebo.selfCollide.value

        if urdf.visuals is not None:
            for visual in urdf.visuals:
                sdf_visual = urdf2sdf(visual)
                sdf.add_visual(sdf_visual.name, sdf_visual)

        if urdf.collisions is not None:
            for collision in urdf.collisions:
                sdf_collision = urdf2sdf(collision)                

                if urdf.gazebo is not None:
                    if urdf.gazebo.maxContacts:
                        sdf_collision.max_contacts = urdf.gazebo.maxContacts.value
                    if urdf.gazebo.mu1: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.friction:
                            sdf_collision.surface.friction = create_sdf_element('friction')
                            sdf_collision.surface.friction.reset(with_optional_elements=True)

                        sdf_collision.surface.friction.ode.mu = urdf.gazebo.mu1.value
                        sdf_collision.surface.friction.bullet.friction = urdf.gazebo.mu1.value
                    if urdf.gazebo.mu2: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.friction:
                            sdf_collision.surface.friction = create_sdf_element('friction')
                            sdf_collision.surface.friction.reset(with_optional_elements=True)
                        
                        sdf_collision.surface.friction.ode.mu2 = urdf.gazebo.mu2.value
                        sdf_collision.surface.friction.bullet.friction2 = urdf.gazebo.mu2.value
                    if urdf.gazebo.kp: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element('contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)
                            
                        sdf_collision.surface.contact.ode.kp = urdf.gazebo.kp.value
                        sdf_collision.surface.contact.bullet.kp = urdf.gazebo.kp.value
                    if urdf.gazebo.kd: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element('contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.kd = urdf.gazebo.kd.value
                        sdf_collision.surface.contact.bullet.kd = urdf.gazebo.kd.value
                    if urdf.gazebo.minDepth: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element('contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)
                        
                        sdf_collision.surface.contact.ode.min_depth = urdf.gazebo.minDepth.value
                    if urdf.gazebo.maxVel: 
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element('surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element('contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.max_vel = urdf.gazebo.maxVel.value                    
                    
                sdf.add_collision(sdf_collision.name, sdf_collision)

    elif urdf._NAME == 'robot':
        sdf.name = urdf.name 

        if urdf.links is not None:
            for link in urdf.links:                
                sdf_link = urdf2sdf(link)
                sdf.add_link(sdf_link.name, sdf_link)
        
        if urdf.joints is not None:
            for joint in urdf.joints:
                sdf_joint = urdf2sdf(joint)
                sdf.add_joint(sdf_joint.name, sdf_joint)

    return sdf
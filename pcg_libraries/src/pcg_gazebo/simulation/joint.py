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
from ..parsers.sdf import Joint as JointSDF
from ..parsers.sdf import create_sdf_element
from .properties import Axis, Pose
from ..log import PCG_ROOT_LOGGER


class Joint(object):
    def __init__(self, 
        name='joint', 
        parent=None, 
        child=None, 
        joint_type='fixed',
        axis_xyz=[0, 0, 1],
        damping=0, 
        friction=0, 
        spring_reference=0, 
        spring_stiffness=0,
        lower=-1e16, 
        upper=1e16, 
        velocity=-1, 
        effort=-1):        
        assert isinstance(name, str), 'Name must be a string'
        assert len(name) > 0, 'Name cannot be an empty string'
        self._name = name

        self._pose = Pose()

        assert isinstance(parent, str), \
            'Parent must be a string, received={}'.format(type(parent))
        assert len(parent) > 0, 'Parent cannot be an empty string'
        self._parent = parent

        assert isinstance(child, str), \
            'Child must be a string, received={}'.format(type(child))
        assert len(child) > 0, 'Child cannot be an empty string'
        self._child = child

        assert joint_type in JointSDF._JOINT_OPTIONS, \
            'Invalid joint type, options={}'.format(JointSDF._JOINT_OPTIONS)
        self._type = joint_type

        if self._type != 'fixed':
            self._axis = [Axis()]            
        elif self._type in ['universal', 'revolute2']:
            self._axis = [Axis(), Axis()]
        else:
            self._axis = list()

        self.set_axis_xyz(axis_xyz)
        self.set_axis_dynamics(
            damping=damping, 
            friction=friction,
            spring_reference=spring_reference,
            spring_stiffness=spring_stiffness
        )
        self.set_axis_limits(
            lower=lower,
            upper=upper,
            velocity=velocity,
            effort=effort
        )
        
    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert isinstance(value, str), \
            'Name must be a string'
        assert len(value) > 0, 'Name cannot be an empty string'
        self._name = value

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        assert isinstance(value, str), 'Parent must be a string'
        assert len(value) > 0, 'Parent cannot be an empty string'
        self._parent = value

    @property
    def child(self):
        return self._child

    @child.setter
    def child(self, value):
        assert isinstance(value, str), 'Child must be a string'
        assert len(value) > 0, 'Child cannot be an empty string'
        self._child = value

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        assert value in JointSDF._JOINT_OPTIONS, \
            'Invalid joint type, options={}'.format(JointSDF._JOINT_OPTIONS)
        self._type = value

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        import collections
        assert isinstance(vec, collections.Iterable), \
            'Input pose vector must be iterable'
        assert len(vec) == 6 or len(vec) == 7, \
            'Pose must be given as position and Euler angles (x, y, z, ' \
            'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
            'qw, qx, qy, qz)'
        for item in vec:
            assert isinstance(item, float) or isinstance(item, int), \
                'All elements in pose vector must be a float or an integer'        
        
        self._pose = Pose(pos=vec[0:3], rot=vec[3::])
        
    @property
    def axes(self):
        return self._axis

    def set_axis_xyz(self, xyz=[0, 0, 1], index=0):
        if self._type == 'fixed':
            PCG_ROOT_LOGGER.warning('Axis vector is ignored for fixed joints')
            return False
        self._axis[index].set_axis(xyz)
        return True

    def set_axis_limits(self, lower=0, upper=0, velocity=-1, effort=-1, index=0):
        if self._type == 'fixed':
            PCG_ROOT_LOGGER.warning('Fixed joints have no limits')
            return False
        if index != 0 and self._type not in ['universal', 'revolute2']:
            PCG_ROOT_LOGGER.warning('Only joints of types universal and revolute2')
            return False

        self._axis[index].set_limits(lower, upper, velocity, effort)
        return True

    def set_axis_dynamics(self, damping=0, friction=0, spring_reference=0, 
        spring_stiffness=0, index=0):
        if self._type == 'fixed':
            PCG_ROOT_LOGGER.warning('Fixed joints have no limits')
            return False
        if index != 0 and self._type not in ['universal', 'revolute2']:
            PCG_ROOT_LOGGER.warning('Only joints of types universal and revolute2')
            return False
        
        self._axis[index].set_dynamics(damping, friction, spring_reference, spring_stiffness)
        return True

    def to_sdf(self):
        joint_sdf = JointSDF()
        joint_sdf.name = self._name
        joint_sdf.parent = self._parent
        joint_sdf.child = self._child
        joint_sdf.type = self._type
        joint_sdf.pose = self.pose.to_sdf()
        
        if len(self._axis) > 0 and self._type != 'fixed':
            joint_sdf.axis = self._axis[0].to_sdf()
            if len(self._axis) == 2:
                joint_sdf.axis2 = self._axis[0].to_sdf(is_axis_2=True)
        return joint_sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'joint', 'Input SDF element must be of type joint'
        joint = Joint(
            name=sdf.name, 
            parent=sdf.parent.value, 
            child=sdf.child.value,
            joint_type=sdf.type)

        # Set model pose
        if sdf.pose is not None:
            joint.pose.from_sdf(sdf.pose)

        if sdf.axis is not None:
            if sdf.axis.xyz is not None:
                joint.set_axis_xyz(sdf.axis.xyz.value)
            if sdf.axis.limit is not None:
                joint.set_axis_limits(
                    sdf.axis.limit.lower.value,
                    sdf.axis.limit.upper.value,
                    -1 if sdf.axis.limit.velocity is None else sdf.axis.limit.velocity.value,
                    -1 if sdf.axis.limit.effort is None else sdf.axis.limit.effort.value
                )
            if sdf.axis.dynamics is not None:
                joint.set_axis_dynamics(
                    0 if sdf.axis.dynamics.damping is None else sdf.axis.dynamics.damping.value,
                    0 if sdf.axis.dynamics.friction is None else sdf.axis.dynamics.friction.value,
                    0 if sdf.axis.dynamics.spring_reference is None else sdf.axis.dynamics.spring_reference.value,
                    0 if sdf.axis.dynamics.spring_stiffness is None else sdf.axis.dynamics.spring_stiffness.value
                )
        
        if sdf.axis2 is not None:
            if sdf.axis2.xyz is not None:
                joint.set_axis_xyz(sdf.axis2.xyz.value, index=1)
            if sdf.axis2.limit is not None:
                joint.set_axis_limits(
                    sdf.axis2.limit.lower.value,
                    sdf.axis2.limit.upper.value,
                    -1 if sdf.axis2.limit.velocity is None else sdf.axis2.limit.velocity.value,
                    -1 if sdf.axis2.limit.effort is None else sdf.axis2.limit.effort.value,
                    index=1
                )
            if sdf.axis2.dynamics is not None:
                joint.set_axis_dynamics(
                    0 if sdf.axis2.dynamics.damping is None else sdf.axis2.dynamics.damping.value,
                    0 if sdf.axis2.dynamics.friction is None else sdf.axis2.dynamics.friction.value,
                    0 if sdf.axis2.dynamics.spring_reference is None else sdf.axis2.dynamics.spring_reference.value,
                    0 if sdf.axis2.dynamics.spring_stiffness is None else sdf.axis2.dynamics.spring_stiffness.value
                )

        return joint
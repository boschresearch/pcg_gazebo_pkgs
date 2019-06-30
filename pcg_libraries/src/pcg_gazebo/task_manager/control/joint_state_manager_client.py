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
import rospy
from time import time
import random
from copy import deepcopy
from pcg_msgs.msg import JointProperties
from pcg_msgs.srv import SetJointPosition, GetJointPosition, \
    SetJointProperties, GetJointProperties
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties


class JointStateManagerClient:
    _LABEL = 'joint_state_manager'
    TAG = 'joint_state_manager'
    INSTANCE = None

    def __init__(self):
        # Method init must be called before the joint methods
        # can be used
        self.is_init = False
        self._ros_namespace = None

    @classmethod
    def get_instance(cls):
        if cls.INSTANCE is None:
            cls.INSTANCE = JointStateManagerClient()
        return cls.INSTANCE

    def init(self, ros_namespace='pcg'):
        assert len(ros_namespace) > 0, "ROS namespace cannot be empty"
        
        self._ros_namespace = ros_namespace
        if self._ros_namespace[0] != '/':
            self._ros_namespace = '/' + self._ros_namespace

        rospy.loginfo("[%s] ROS namespace=%s",
                      self.__class__.__name__, self._ros_namespace)
        
        rospy.loginfo('Wait for Gazebo to load world...')
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=60)
        rospy.loginfo('Gazebo is running!')
       
        tag = deepcopy(self._ros_namespace)
        if tag[-1] != '/':
            tag += '/'
        tag += 'modules'
        assert rospy.has_param(tag), 'No PCG modules listed, key=' + tag

        modules_info = rospy.get_param(tag)

        assert self._LABEL in modules_info, \
            'Joint state manager server was not initialized'

        assert 'type' in modules_info[self._LABEL], 'No type found for module'

        self._ros_namespace_prefix = self._ros_namespace
        if self._ros_namespace_prefix[-1] != '/':
            self._ros_namespace_prefix += '/'

        self._ros_namespace_prefix += self._LABEL

        # Load the service handlers from the PCG joint state manager
        srv_names = [
            'set_joint_position',
            'get_joint_position',
            'set_joint_properties',
            'get_joint_properties'
        ]

        srv_classes = [
            SetJointPosition,
            GetJointPosition,
            SetJointProperties,
            GetJointProperties
        ]

        self.services = dict()
        for name, srv_class in zip(srv_names, srv_classes):
            srv_name = self._ros_namespace_prefix + '/' + name

            # Check if service exists
            rospy.wait_for_service(srv_name, timeout=10)

            # Load service proxy
            self.services[name] = rospy.ServiceProxy(srv_name, srv_class)
            rospy.loginfo('%s service loaded successfully', srv_name)

        # Load the service handlers from the PCG joint state manager
        srv_names = [
            'get_world_properties',
            'get_model_properties'
        ]

        srv_classes = [
            GetWorldProperties,
            GetModelProperties
        ]

        for name, srv_class in zip(srv_names, srv_classes):
            srv_name = '/gazebo/' + name
            # Check if service exists
            rospy.wait_for_service(srv_name, timeout=10)

            # Load service proxy
            self.services[name] = rospy.ServiceProxy(srv_name, srv_class)
            rospy.loginfo('%s service loaded successfully', srv_name)

        self.is_init = True

    def get_models(self):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None
        world_props = self.services['get_world_properties']()
        return world_props.model_names

    def get_joints(self, model_name=None, exclude_fixed_joints=True):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None
        if model_name is None:
            # Get list of all joints in the simulation grouped by model
            models = self.get_models()
            if len(models) == 0:
                return None

        else:
            if model_name not in self.get_models():
                return None

            models = [model_name]

        output = dict()
        for model in models:
            model_props = self.services['get_model_properties'](model)

            joints = model_props.joint_names
            output[model] = list()

            if exclude_fixed_joints:
                for joint in joints:
                    joint_props = self.services['get_joint_properties'](
                        model_name=model, joint_name=joint)
                    if joint_props.joint_properties.type != JointProperties.FIXED:
                        output[model].append(joint)
            else:
                output[model] = joints
        return output

    def joint_exists(self, model_name, joint_name):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None
        if model_name not in self.get_models():
            rospy.logerr('Model {} does not exist'.format(model_name))
            return False

        joints = self.get_joints(model_name)
        
        if joint_name not in joints[model_name]:
            rospy.logerr('Joint {} not found for model {}'.format(joint_name, model_name))
            return False

        return True

    def get_joint_limits(self, model_name, joint_name):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None, None
        if not self.joint_exists(model_name, joint_name):
            return None, None

        joint_props = self.services['get_joint_properties'](
            model_name=model_name, joint_name=joint_name)
        
        return joint_props.joint_properties.lower_lim, \
               joint_props.joint_properties.upper_lim

    def get_joint_dof(self, model_name, joint_name):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None
        lower_lim, = self.get_joint_limits()
        if lower_lim is None:
            rospy.logerr('Error while retrieving joint limits')
            return None

        return len(lower_lim)

    def set_joint_positions(self, model_name, joint_name, positions):
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None, None
        if not self.joint_exists(model_name, joint_name):
            return None, None

        joint_limits = self.get_joint_limits(model_name, joint_name)

        if len(positions) != len(joint_limits[0]):
            rospy.logerr(
                'Joint {} has {} DOFs, but only {} joint positions were given'.format(
                    joint_name, len(joint_limits[0]), len(positions)))
            return False

        joint_positions = list()        
        for p, lower_lim, upper_lim in zip(positions, joint_limits[0], joint_limits[1]):            
            # Make sure that the joint positions are found within the 
            # lower and upper limits
            joint_positions.append(max(lower_lim, min(upper_lim, p)))        

        resp = self.services['set_joint_position'](
            joint_name=joint_name,
            model_name=model_name,
            position=joint_positions)

        return resp.success

    def set_random_joint_positions(self, model_name, joint_name, lower_lim=None, upper_lim=None):        
        if not self.is_init:
            rospy.logerr('Joint state control client has not been initialized')
            return None
        if not self.joint_exists(model_name, joint_name):
            return False

        ll, ul = self.get_joint_limits(model_name, joint_name)

        if lower_lim is None:
            lower_lim = ll
        else:
            assert isinstance(lower_lim, list), 'Lower limit vector must be a list'
            assert len(lower_lim) == len(ll), \
                'Lower limit vector must have the same length as the number of DOFs'
            
        if upper_lim is None:
            upper_lim = ul
        else:
            assert isinstance(upper_lim, list), 'Upper limit vector must be a list'
            assert len(upper_lim) == len(ul), \
                'Upper limit vector must have the same length as the number of DOFs'

        # Initializing seed
        random.seed(time())

        random_pos_vec = list()
        for i in range(len(lower_lim)):
            random_pos_vec.append(random.random() * (upper_lim[i] - lower_lim[i]))
            random_pos_vec[-1] += lower_lim[i]

        self.set_joint_positions(model_name, joint_name, random_pos_vec)
        return True

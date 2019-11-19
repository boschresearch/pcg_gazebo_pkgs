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
from ...log import PCG_ROOT_LOGGER
from ...simulation import SimulationModel, Link
from ...simulation.properties import Collision, Visual
import numpy as np


class HingedDoor(SimulationModel):
    _HAND_CONVENTIONS = ['LH', 'RH', 'LHR', 'RHR']

    def __init__(self, door_mesh_filename=None, door_gazebo_model=None, 
        width=0.6, thickness=0.04, height=2.0, mass=10,
        set_origin_to_ground=True, fix_to_world=True, 
        hand_convention='LH', max_opening_angle=np.pi/2, 
        model_name='door', frame_mesh_filename=None, 
        frame_gazebo_model=None, with_frame=True, frame_width=0.04,
        frame_height=0.04, frame_depth=0.04):
        super(HingedDoor, self).__init__()
        self._hand_convention = hand_convention
        self._max_opening_angle = max_opening_angle
        # Name of the door model
        self.name = model_name
        # Name of the links 
        self._hinge_joint_name = 'hinge'
        self._door_link_name = 'door'
        self._frame_link_name = 'frame'

        if door_mesh_filename is not None:
            self.create_door_from_mesh(door_mesh_filename)
        elif door_gazebo_model is not None:
            self.create_door_from_gazebo_model(door_gazebo_model)
        else:
            self.create_rectangular_door_link(
                width=width,
                thickness=thickness,
                height=height,
                mass=mass,
                set_origin_to_ground=set_origin_to_ground)

        if with_frame:
            if frame_mesh_filename is not None:
                pass
            elif frame_gazebo_model is not None:
                pass
            else:
                self.create_frame(
                    width=frame_width, 
                    height=frame_height, 
                    depth=frame_depth)

    @property
    def hand_conventions(self):
        return self._HAND_CONVENTIONS

    def create_door_from_mesh(self, mesh):
        pass

    def create_door_from_gazebo_model(self, gazebo_model):
        pass

    def create_rectangular_door_link(self, width=0.6, thickness=0.04, height=2.0, 
        mass=10, set_origin_to_ground=True):
        assert width > 0, 'Door width must be greater than zero'
        assert thickness > 0, 'Door thickness must be greater than zero'
        assert height > 0, 'Door height must be greater than zero'
        assert mass > 0, 'Door mass must be greater than zero'

        if self._door_link_name in self.link_names:
            self.rm_link(self._door_link_name)        

        self.add_cuboid_link(
            link_name=self._door_link_name,
            mass=mass,
            size=[thickness, width, height])

        if self._hinge_joint_name in self.joint_names:
            self.rm_joint(self._hinge_joint_name)

        if self._hand_convention in ['LH', 'LHR']:
            joint_pose = [0, width / 2, 0, 0, 0, 0]
        elif self._hand_convention in ['RH', 'RHR']:
            joint_pose = [0, -width / 2, 0, 0, 0, 0]
        else: 
            joint_pose = [0 for _ in range(6)]

        if self._hand_convention in ['LH', 'RHR']:
            axis_limits = dict(lower=0, upper=self._max_opening_angle)
        elif self._hand_convention in ['RH', 'LHR']:
            axis_limits = dict(lower=-self._max_opening_angle, upper=0)

        self.add_joint(
            name=self._hinge_joint_name,
            parent='world',
            child=self._door_link_name,
            pose=joint_pose,
            joint_type='revolute',            
            axis_limits=axis_limits)

        if set_origin_to_ground:
            self.pose = [0, 0, height / 2, 0, 0, 0]

    def create_frame(self, width=0.04, height=0.04, 
        depth=0.04):
        assert self._door_link_name in self._links, 'Door link was not created'

        if self._frame_link_name in self._links:
            self.rm_link(self._frame_link_name)

        frame_link = Link(name=self._frame_link_name)

        # Get door bounds
        door_bounds = self.links[self._door_link_name].get_bounds()

        # Left frame
        frame_size = [
            depth, 
            width, 
            door_bounds[1, 2] - door_bounds[0, 2]]
                
        side_collision = Collision(name='left_frame_collision')
        side_collision.set_box_as_geometry(frame_size)

        side_visual = Visual(name='left_frame_visual')
        side_visual.set_box_as_geometry(frame_size)
        
        # Add left frame
        frame_link.add_collision(Collision(
            name='left_frame_collision',
            pose=[0, door_bounds[0, 1] - frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        frame_link.add_visual(Visual(
            name='left_frame_visual',
            pose=[0, door_bounds[0, 1] - frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        # Add right frame
        frame_link.add_collision(Collision(
            name='right_frame_collision',
            pose=[0, door_bounds[1, 1] + frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        frame_link.add_visual(Visual(
            name='right_frame_visual',
            pose=[0, door_bounds[1, 1] + frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        # Add top frame
        # Calculate geometry of top frame
        frame_size = [
            depth, 
            door_bounds[1, 1] - door_bounds[0, 1] + 2 * width, 
            height]

        frame_link.add_collision(Collision(
            name='top_frame_collision',
            pose=[0, 0, door_bounds[1, 2] + frame_size[2] / 2, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))
        frame_link.add_visual(Visual(
            name='top_frame_visual',
            pose=[0, 0, door_bounds[1, 2] + frame_size[2] / 2, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        self.add_link(name='frame', link=frame_link)

        self.add_joint(
            name=self._frame_link_name + '_to_world',
            parent='world',
            child=frame_link.name,
            pose=[0, 0, 0, 0, 0, 0],
            joint_type='revolute',            
            axis_limits=dict(lower=0, upper=0))






        
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
import trimesh
from ..log import PCG_ROOT_LOGGER


class CollisionChecker(object):
    """Mesh-based collision checker manager. The meshes can be 
    added to the collision check scene and used to check any 
    other object for collisions.
    
    > *Input arguments*
    
    * `ignore_ground_plane` (*type:* `bool`, *value:* `True`): Ignores the meshes flagged as ground plane when performing collision checks.    
    """
    def __init__(self, ignore_ground_plane=True):
        self._fixed_models = list()
        self._simulation_scenario = trimesh.scene.Scene()
        self._ignore_ground_plane = ignore_ground_plane
        PCG_ROOT_LOGGER.info('Collision checker created')

    @property
    def scene(self):
        """`trimesh.scene.Scene`: Collision check scenario"""
        return self._simulation_scenario

    @property
    def fixed_models(self):
        return self._fixed_models

    @property
    def n_fixed_models(self):
        return len(self._fixed_models)

    @property
    def n_meshes(self):
        return len(self._simulation_scenario.dump())

    def reset_scenario(self):
        """Remove all meshes from collision check scene."""
        self._simulation_scenario = trimesh.scene.Scene()
        PCG_ROOT_LOGGER.info('Collision checker scenario is now empty')

    def reset_to_fixed_model_scenario(self):
        """Remove all meshes that were not generated by a fixed-pose engine."""
        self.reset_scenario()        
        for model in self._fixed_models:
            meshes = model.get_meshes(mesh_type='collision')
            self._simulation_scenario.add_geometry(meshes)
        PCG_ROOT_LOGGER.info(
            'Collision checker scenario restarted with all'
            ' fixed-models={}'.format([model.name for model in self._fixed_models]))

    def add_fixed_model(self, model):
        """Add a model as a fixed-pose model to the scene.
        
        > *Input arguments*
        
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure        
        """
        if model.is_ground_plane and self._ignore_ground_plane:
            PCG_ROOT_LOGGER.info(
                'Model <{}> is a ground plane,'
                ' ignoring it for collision checking'.format(model.name))
            return

        self._fixed_models.append(model)
        self.add_model(model)

    def add_model(self, model):
        """Add model to collision checking scene.
        
        > *Input arguments*
        
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure                
        """
        if model.is_ground_plane and self._ignore_ground_plane:
            return

        meshes = model.get_meshes(mesh_type='collision')
        self._simulation_scenario.add_geometry(meshes)


    def show(self):
        """Display the current collision check scenario using `pyglet`."""
        if not self._simulation_scenario.is_empty:
            self._simulation_scenario.show()
        else:
            PCG_ROOT_LOGGER.warning('Collision scene is empty')

    def check_collision_with_current_scene(self, model):
        """Check if there are any collisions between `model` and 
        the meshes in the scene.
        
        > *Input arguments*
        
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure                
        
        > *Returns*
        
        `True`, if any collision is detected. `False`, otherwise.
        """
        manager, objects = trimesh.collision.scene_to_collision(
            self._simulation_scenario)

        scene_meshes = self._simulation_scenario.dump()

        meshes = model.get_meshes(mesh_type='collision')
        for mesh in meshes:
            if manager.in_collision_single(mesh):
                return True
            # Test if mesh in inside another mesh in the collision
            # manager scene
            # It will only work for scene meshes that are watertight
            for scene_mesh in scene_meshes:
                # Check if a scene mesh contains the model mesh to be tested
                if scene_mesh.is_watertight:                    
                    if scene_mesh.contains(mesh.vertices).any():
                        return True
                # Check if the model mesh being tested contains any of the
                # scene meshes
                if mesh.is_watertight:
                    if mesh.contains(scene_mesh.vertices).any():
                        return True
        return False

    def check_for_collisions(self):
        """Check if there are any collisions amongst the meshes in the scene.
                
        > *Returns*
        
        `True`, if any collision is detected. `False`, otherwise.
        """
        manager, objects = trimesh.collision.scene_to_collision(
            self._simulation_scenario)
        return manager.in_collision_internal()


class SingletonCollisionChecker(CollisionChecker):
    """Singleton collision checker that can be have one instance accessed by
    multiple clients. It facilitates sharing the collision managar amongst 
    many engines, for example.
    
    > *Attributes*
    
    * `INSTANCE` (*type:* `SingletonCollisionChecker`, *value:* `None`): Instance 
    of the singleton collision checker that is initialized by the first call of 
    `get_instance()`.
    
    > *Input arguments*
    
    * `ignore_ground_plane` (*type:* `bool`, *value:* `True`): Ignores the meshes 
    flagged as ground plane when performing collision checks.    
    """
    INSTANCE = None

    def __init__(self, ignore_ground_plane=True):
        CollisionChecker.__init__(self, ignore_ground_plane)

    @staticmethod
    def get_instance(**kwargs):
        """Return a singleton instance of the collision checker.
        
        > *Input arguments*
        
        * `kwargs` (*type:* `dict`): Input arguments for the `SingletonCollisionChecker` 
        instance.
        
        > *Returns*
        
        A `SingletonCollisionChecker` instance
        """
        if SingletonCollisionChecker.INSTANCE is None:
            SingletonCollisionChecker.INSTANCE = SingletonCollisionChecker(kwargs)
        return SingletonCollisionChecker.INSTANCE

    
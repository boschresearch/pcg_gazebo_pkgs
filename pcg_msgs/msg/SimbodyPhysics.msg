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

# List of parameters for the Simbody physics engine:
# https://bitbucket.org/osrf/gazebo/src/bd1cdea8fa3d71d6afcdaa83c9b01891d5f72e71/gazebo/physics/simbody/SimbodyPhysics.cc?at=default&fileviewer=file-view-default

# Maximum physics step size when physics update step must return
float64 max_step_size
bool auto_disable_bodies           # enable auto disabling of bodies, default false
float64 accuracy
float64 max_transient_velocity
float64 stiffness
float64 dissipation
float64 plastic_coef_restitution
float64 plastic_impact_velocity
float64 static_friction
float64 viscous_friction
float64 override_impact_capture_velocity
float64 override_stiction_transition_velocity
string solver_type

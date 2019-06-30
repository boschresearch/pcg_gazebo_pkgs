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

# List of parameters for the Bullet physics engine:
# https://bitbucket.org/osrf/gazebo/src/bd1cdea8fa3d71d6afcdaa83c9b01891d5f72e71/gazebo/physics/bullet/BulletPhysics.cc?at=default&fileviewer=file-view-default

# Minimum internal step size
float64 min_step_size
# Snable auto disabling of bodies, default false
bool auto_disable_bodies
# Solver used by the engine
string solver_type
# Number of LCP PGS iterations. If sor_lcp_tolerance is negative,
# full iteration count is executed. Otherwise, PGS may stop iteration
# early if sor_lcp_tolerance is satisfied by the total RMS residual
uint32 iters
# Relaxation parameter for Projected Gauss-Seidel (PGS) updates
float64 sor
# Global ERP
float64 erp
# Global CFM
float64 cfm

float64 contact_surface_layer

bool split_impulse

float64 split_impulse_penetration_threshold

uint32 max_contacts

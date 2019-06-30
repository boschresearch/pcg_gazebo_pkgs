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

# Joint type
uint8 type
uint8 REVOLUTE    = 0                # single DOF
uint8 CONTINUOUS  = 1                # single DOF (revolute w/o joints)
uint8 PRISMATIC   = 2                # single DOF
uint8 FIXED       = 3                # 0 DOF
uint8 BALL        = 4                # 3 DOF
uint8 UNIVERSAL   = 5                # 2 DOF
# Dynamics properties
float64[] damping
float64[] stiffness
# Joint state
float64[] position
float64[] rate
# Joint limits
float64[] lower_lim
float64[] upper_lim
# Velocity limits
float64[] vel_lim
# Effort limits
float64[] effort_lim
# Joint name
string joint_name
# Model name
string model_name

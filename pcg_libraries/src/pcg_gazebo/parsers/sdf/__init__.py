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
import inspect

from ..types import XMLBase

from .accel import Accel
from .accuracy import Accuracy
from .actor import Actor
from .allow_auto_disable import AllowAutoDisable
from .altimeter import Altimeter
from .always_on import AlwaysOn
from .ambient import Ambient
from .angular_velocity import AngularVelocity
from .animation import Animation
from .attenuation import Attenuation
from .auto_start import AutoStart
from .axis import Axis
from .axis2 import Axis2
from .bias_mean import BiasMean
from .bias_stddev import BiasStdDev
from .bounce import Bounce
from .box import Box
from .bullet import Bullet
from .camera import Camera
from .cast_shadows import CastShadows
from .category_bitmask import CategoryBitmask
from .center import Center
from .cfm import CFM
from .child import Child
from .clip import Clip
from .coefficient import Coefficient
from .collide_bitmask import CollideBitmask
from .collide_without_contact_bitmask import CollideWithoutContactBitmask
from .collide_without_contact import CollideWithoutContact
from .collision import Collision
from .constant import Constant
from .constraints import Constraints
from .contact_max_correcting_vel import ContactMaxCorrectingVel
from .contact_surface_layer import ContactSurfaceLayer
from .contact import Contact
from .cylinder import Cylinder
from .damping import Damping
from .delay_start import DelayStart
from .depth_camera import DepthCamera
from .diffuse import Diffuse
from .direction import Direction
from .dissipation import Dissipation
from .distortion import Distortion
from .dynamic_friction import DynamicFriction
from .dynamics import Dynamics
from .effort import Effort
from .elastic_modulus import ElasticModulus
from .emissive import Emissive
from .empty import Empty
from .enable_wind import EnableWind
from .erp import ERP
from .falloff import FallOff
from .far import Far
from .fdir1 import FDir1
from .filename import Filename
from .force_torque import ForceTorque
from .format import Format
from .frame import Frame
from .friction_model import FrictionModel
from .friction import Friction
from .friction2 import Friction2
from .geometry import Geometry
from .granularity import Granularity
from .gravity import Gravity
from .height import Height
from .horizontal_fov import HorizontalFOV
from .horizontal import Horizontal
from .image import Image
from .imu import IMU
from .include import Include
from .inertia import Inertia
from .inertial import Inertial
from .initial_position import InitialPosition
from .inner_angle import InnerAngle
from .interpolate_x import InterpolateX
from .iters import Iters
from .ixx import IXX
from .ixy import IXY
from .ixz import IXZ
from .iyy import IYY
from .iyz import IYZ
from .izz import IZZ
from .joint import Joint
from .k1 import K1
from .k2 import K2
from .k3 import K3
from .kd import Kd
from .kinematic import Kinematic
from .kp import Kp
from .laser_retro import LaserRetro
from .length import Length
from .light import Light
from .lighting import Lighting
from .limit import Limit
from .linear_acceleration import LinearAcceleration
from .linear import Linear
from .link import Link
from .localization import Localization
from .loop import Loop
from .lower import Lower
from .mass import Mass
from .material import Material
from .max_angle import MaxAngle
from .max_contacts import MaxContacts
from .max_step_size import MaxStepSize
from .min import Min
from .max_transient_velocity import MaxTransientVelocity
from .max_vel import MaxVel
from .max import Max
from .mean import Mean
from .measure_direction import MeasureDirection
from .mesh import Mesh
from .min_angle import MinAngle
from .min_depth import MinDepth
from .min_step_size import MinStepSize
from .model import Model
from .mu import Mu
from .mu2 import Mu2
from .must_be_loop_joint import MustBeLoopJoint
from .name import Name
from .near import Near
from .noise import Noise
from .normal_map import NormalMap
from .normal import Normal
from .ode import ODE
from .orientation_reference_frame import OrientationReferenceFrame
from .outer_angle import OuterAngle
from .output import Output
from .override_impact_capture_velocity import OverrideImpactCaptureVelocity
from .override_stiction_transition_velocity import OverrideStictionTransitionVelocity
from .p1 import P1
from .p2 import P2
from .parent import Parent
from .patch_radius import PatchRadius
from .path import Path
from .physics import Physics
from .plane import Plane
from .plastic_coef_restitution import PlasticCoefRestitution
from .plastic_impact_velocity import PlasticImpactVelocity
from .plugin import Plugin
from .point import Point
from .poissons_ratio import PoissonsRatio
from .polyline import Polyline
from .pose import Pose
from .precision import Precision
from .precon_iters import PreConIters
from .provide_feedback import ProvideFeedback
from .quadratic import Quadratic
from .radius import Radius
from .range import Range
from .rate import Rate
from .ray import Ray
from .real_time_factor import RealTimeFactor
from .real_time_update_rate import RealTimeUpdateRate
from .resolution import Resolution
from .restitution_coefficient import RestitutionCoefficient
from .rolling_friction import RollingFriction
from .samples import Samples
from .save import Save
from .scale import Scale
from .scan import Scan
from .scene import Scene
from .script import Script
from .sdf import SDF
from .self_collide import SelfCollide
from .sensor import Sensor
from .shader import Shader
from .simbody import Simbody
from .size import Size
from .skin import Skin
from .sky import Sky
from .slip import Slip
from .slip1 import Slip1
from .slip2 import Slip2
from .soft_cfm import SoftCFM
from .soft_erp import SoftERP
from .solver import Solver
from .sor import Sor
from .specular import Specular
from .sphere import Sphere
from .split_impulse_penetration_threshold import SplitImpulsePenetrationThreshold
from .split_impulse import SplitImpulse
from .spot import Spot
from .spring_reference import SpringReference
from .spring_stiffness import SpringStiffness
from .static_friction import StaticFriction
from .static import Static
from .stddev import StdDev
from .stiffness import Stiffness
from .submesh import SubMesh
from .sunrise import Sunrise
from .sunset import Sunset
from .surface_radius import SurfaceRadius
from .surface import Surface
from .threshold import Threshold
from .time import Time
from .topic import Topic
from .torsional import Torsional
from .trajectory import Trajectory
from .transparency import Transparency
from .type import Type
from .update_rate import UpdateRate
from .upper import Upper
from .urdf import URDF
from .uri import URI
from .use_dynamic_moi_rescaling import UseDynamicMOIRescaling
from .use_parent_model_frame import UseParentModelFrame
from .use_patch_radius import UsePatchRadius
from .velocity import Velocity
from .vertical_position import VerticalPosition
from .vertical_velocity import VerticalVelocity
from .vertical import Vertical
from .viscous_friction import ViscousFriction
from .visual import Visual
from .visualize import Visualize
from .waypoint import Waypoint
from .width import Width
from .world import World
from .x import X
from .xyz import XYZ
from .y import Y
from .z import Z


def get_all_sdf_element_classes():
    output = list()
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase) and obj._TYPE == 'sdf':
                output.append(obj)
    return output


def create_sdf_element(tag, *args):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf':
                    return obj(*args)
    return None


def create_sdf_type(tag):
    module_name = os.path.dirname(os.path.realpath(__file__))
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf':
                    return obj
    return None


def is_sdf_element(obj):
    return obj.__class__ in XMLBase.__subclasses__() and obj._TYPE == 'sdf'

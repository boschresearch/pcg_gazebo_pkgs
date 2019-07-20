# pcg_gazebo.parsers.sdf

# Accel
```python
Accel(self)
```
Noise parameters for linear accelerations.

> *Child elements*

* `mean`
* `stddev`
* `bias_mean`
* `bias_stddev`

> SDF versions

* `1.4`

> *Source*

* [`<accel>` (SDF 1.4)](http://sdformat.org/spec?elem=sensor&ver=1.4)

## bias_mean
Return the bias mean value SDF element,
to read the value use `obj.bias_mean.value`

## bias_stddev
Return the bias standard deviation value SDF element,
to read the value use `obj.bias_stddev.value`

## mean
Return the mean value SDF element, to read the value use
`obj.mean.value`

## stddev
Return the standard deviation value SDF element,
to read the value use `obj.stddev.value`

# Accuracy
```python
Accuracy(self, default=0.001)
```

# Actor
```python
Actor(self)
```

# AllowAutoDisable
```python
AllowAutoDisable(self, default=False)
```

# Altimeter
```python
Altimeter(self)
```

# AlwaysOn
```python
AlwaysOn(self, default=False)
```

# Ambient
```python
Ambient(self, size=4)
```

# AngularVelocity
```python
AngularVelocity(self)
```

# Animation
```python
Animation(self)
```

# Attenuation
```python
Attenuation(self)
```

# AutoStart
```python
AutoStart(self, default=True)
```

# Axis
```python
Axis(self)
```

# Axis2
```python
Axis2(self)
```

# BiasMean
```python
BiasMean(self, default=0)
```

# BiasStdDev
```python
BiasStdDev(self, default=0)
```

# Bounce
```python
Bounce(self)
```

# Box
```python
Box(self)
```

# Bullet
```python
Bullet(self, mode='physics')
```

# Camera
```python
Camera(self)
```

# CastShadows
```python
CastShadows(self, default=True)
```

# Center
```python
Center(self, default=False)
```

# CFM
```python
CFM(self, default=0)
```

# Child
```python
Child(self, default='none')
```

# Clip
```python
Clip(self)
```

# Coefficient
```python
Coefficient(self, default=1)
```

# Collision
```python
Collision(self)
```

# Constant
```python
Constant(self, default=0)
```

# Constraints
```python
Constraints(self, engine='ode')
```

# ContactMaxCorrectingVel
```python
ContactMaxCorrectingVel(self, default=100)
```

# ContactSurfaceLayer
```python
ContactSurfaceLayer(self, default=0.001)
```

# Contact
```python
Contact(self, mode='simbody')
```

# Cylinder
```python
Cylinder(self)
```

# Damping
```python
Damping(self, default=0)
```

# DelayStart
```python
DelayStart(self, default=0)
```

# DepthCamera
```python
DepthCamera(self)
```

# Diffuse
```python
Diffuse(self, default=[0, 0, 0, 1])
```

# Direction
```python
Direction(self, default=[0, 0, -1])
```

# Dissipation
```python
Dissipation(self, default=100)
```

# Distortion
```python
Distortion(self)
```

# DynamicFriction
```python
DynamicFriction(self, default=0.9)
```

# Dynamics
```python
Dynamics(self)
```

# Effort
```python
Effort(self, default=-1)
```

# Emissive
```python
Emissive(self, size=4)
```

# Empty
```python
Empty(self)
```

# EnableWind
```python
EnableWind(self, default=False)
```

# ERP
```python
ERP(self, default=0.2)
```

# FallOff
```python
FallOff(self, default=0)
```

# Far
```python
Far(self, default=0)
```

# FDir1
```python
FDir1(self, default=[0, 0, 0])
```

3-tuple specifying direction of mu1 in the collision local reference frame

Args:
    default (list): Direction unit vector

Attributes:
    value (list): Stored direction unit vector

# Filename
```python
Filename(self, default='none')
```

# ForceTorque
```python
ForceTorque(self)
```

# Format
```python
Format(self, default='R8G8B8')
```

# Frame
```python
Frame(self, default='')
```

# FrictionModel
```python
FrictionModel(self)
```

# Friction
```python
Friction(self)
```

Configuration of the collision friction parameters.

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction

# Friction2
```python
Friction2(self, default=1)
```

# Geometry
```python
Geometry(self)
```

# Granularity
```python
Granularity(self, default=1)
```

# Gravity
```python
Gravity(self, default=True)
```

# Height
```python
Height(self, default=1)
```

# HorizontalFOV
```python
HorizontalFOV(self, default=1.047)
```

# Horizontal
```python
Horizontal(self)
```

# Image
```python
Image(self, mode='geometry')
```

# IMU
```python
IMU(self)
```

# Include
```python
Include(self)
```

# Inertia
```python
Inertia(self)
```

# Inertial
```python
Inertial(self)
```

# InitialPosition
```python
InitialPosition(self, default=0)
```

# InnerAngle
```python
InnerAngle(self, default=0)
```

# InterpolateX
```python
InterpolateX(self, default=False)
```

# Iters
```python
Iters(self, default=50)
```

# IXX
```python
IXX(self, default=0)
```

# IXY
```python
IXY(self, default=0)
```

# IXZ
```python
IXZ(self, default=0)
```

# IYY
```python
IYY(self, default=0)
```

# IYZ
```python
IYZ(self, default=0)
```

# IZZ
```python
IZZ(self, default=0)
```

# Joint
```python
Joint(self)
```

# K1
```python
K1(self, default=0)
```

# K2
```python
K2(self, default=0)
```

# K3
```python
K3(self, default=0)
```

# Kd
```python
Kd(self, default=1)
```

# Kinematic
```python
Kinematic(self, default=False)
```

# Kp
```python
Kp(self, default=1000000000000.0)
```

# LaserRetro
```python
LaserRetro(self, default=0)
```

# Length
```python
Length(self)
```

# Light
```python
Light(self)
```

# Lighting
```python
Lighting(self, default=False)
```

# Limit
```python
Limit(self, mode='axis')
```

# LinearAcceleration
```python
LinearAcceleration(self)
```

# Linear
```python
Linear(self, default=0)
```

# Link
```python
Link(self)
```

# Localization
```python
Localization(self, default='CUSTOM')
```

# Loop
```python
Loop(self, default=False)
```

# Lower
```python
Lower(self, default=-1e+16)
```

# Mass
```python
Mass(self)
```

# Material
```python
Material(self)
```

# MaxAngle
```python
MaxAngle(self, default=0)
```

# MaxContacts
```python
MaxContacts(self, default=20)
```

# MaxStepSize
```python
MaxStepSize(self, default=0.001)
```

# MaxTransientVelocity
```python
MaxTransientVelocity(self, default=0.01)
```

# MaxVel
```python
MaxVel(self, default=0.01)
```

# Max
```python
Max(self, default=0)
```

# Mean
```python
Mean(self)
```

# MeasureDirection
```python
MeasureDirection(self, default='child_to_parent')
```

# Mesh
```python
Mesh(self)
```

# MinAngle
```python
MinAngle(self, default=0)
```

# MinDepth
```python
MinDepth(self, default=0)
```

# MinStepSize
```python
MinStepSize(self, default=0.0001)
```

# Min
```python
Min(self, default=0)
```

# Model
```python
Model(self)
```

# Mu
```python
Mu(self, default=1)
```

Coefficient of friction in the range of [0, 1]

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction

# Mu2
```python
Mu2(self, default=1)
```

Second coefficient of friction in the range of [0, 1]

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction

# MustBeLoopJoint
```python
MustBeLoopJoint(self, default=False)
```

# Name
```python
Name(self, default='none')
```

# Near
```python
Near(self, default=0)
```

# Noise
```python
Noise(self, type='none')
```

# NormalMap
```python
NormalMap(self, default='')
```

# Normal
```python
Normal(self)
```

# ODE
```python
ODE(self, mode)
```

# OrientationReferenceFrame
```python
OrientationReferenceFrame(self)
```

# OuterAngle
```python
OuterAngle(self, default=0)
```

# Output
```python
Output(self, default='depths')
```

# OverrideImpactCaptureVelocity
```python
OverrideImpactCaptureVelocity(self, default=0.001)
```

# OverrideStictionTransitionVelocity
```python
OverrideStictionTransitionVelocity(self, default=0.9)
```

# P1
```python
P1(self, default=0)
```

# P2
```python
P2(self, default=0)
```

# Parent
```python
Parent(self, default='parent')
```

# PatchRadius
```python
PatchRadius(self, default=0)
```

# Path
```python
Path(self, default='__default__')
```

# Physics
```python
Physics(self, mode='ode')
```

# Plane
```python
Plane(self)
```

# PlasticCoefRestitution
```python
PlasticCoefRestitution(self, default=0.5)
```

# PlasticImpactVelocity
```python
PlasticImpactVelocity(self, default=0.5)
```

# Plugin
```python
Plugin(self, default={})
```

# Point
```python
Point(self, vec_length=2)
```

# Polyline
```python
Polyline(self)
```

# Pose
```python
Pose(self)
```

# Precision
```python
Precision(self)
```

# PreConIters
```python
PreConIters(self, default=0)
```

# ProvideFeedback
```python
ProvideFeedback(self, default=False)
```

# Quadratic
```python
Quadratic(self, default=0)
```

# Radius
```python
Radius(self, default=0)
```

# Range
```python
Range(self)
```

# Rate
```python
Rate(self, type='none')
```

# Ray
```python
Ray(self)
```

# RealTimeFactor
```python
RealTimeFactor(self, default=1)
```

# RealTimeUpdateRate
```python
RealTimeUpdateRate(self, default=1)
```

# Resolution
```python
Resolution(self, default=0)
```

# RestitutionCoefficient
```python
RestitutionCoefficient(self, default=0)
```

# RollingFriction
```python
RollingFriction(self, default=1)
```

# Samples
```python
Samples(self, default=640)
```

# Save
```python
Save(self)
```

# Scale
```python
Scale(self, size=3)
```

# Scan
```python
Scan(self)
```

# Script
```python
Script(self, mode='material')
```

# SDF
```python
SDF(self, mode='world')
```

# SelfCollide
```python
SelfCollide(self, default=False)
```

# Sensor
```python
Sensor(self, mode='altimeter')
```

# Shader
```python
Shader(self)
```

# Simbody
```python
Simbody(self, mode='physics')
```

# Size
```python
Size(self, vec_length=3)
```

# Skin
```python
Skin(self)
```

# Slip
```python
Slip(self, default=0)
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient

# Slip1
```python
Slip1(self, default=0)
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient

# Slip2
```python
Slip2(self, default=0)
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient

# SoftCFM
```python
SoftCFM(self, default=0)
```

# SoftERP
```python
SoftERP(self, default=0.2)
```

# Solver
```python
Solver(self, engine='ode')
```

# Sor
```python
Sor(self, default=1.3)
```

# Specular
```python
Specular(self, default=[0.1, 0.1, 0.1, 1])
```

# Sphere
```python
Sphere(self)
```

# SplitImpulsePenetrationThreshold
```python
SplitImpulsePenetrationThreshold(self, default=-0.01)
```

# SplitImpulse
```python
SplitImpulse(self, default=True)
```

# Spot
```python
Spot(self)
```

# SpringReference
```python
SpringReference(self, default=0)
```

# SpringStiffness
```python
SpringStiffness(self, default=0)
```

# StaticFriction
```python
StaticFriction(self, default=0.9)
```

# Static
```python
Static(self, default=False)
```

# StdDev
```python
StdDev(self)
```

# Stiffness
```python
Stiffness(self, default=100000000.0)
```

# SubMesh
```python
SubMesh(self)
```

# SurfaceRadius
```python
SurfaceRadius(self, default=0)
```

# Surface
```python
Surface(self)
```

# Threshold
```python
Threshold(self, default=0)
```

# Time
```python
Time(self, default=0)
```

# Topic
```python
Topic(self, default='none')
```

# Torsional
```python
Torsional(self)
```

# Trajectory
```python
Trajectory(self)
```

# Transparency
```python
Transparency(self, default=0)
```

# Type
```python
Type(self, default='')
```

# UpdateRate
```python
UpdateRate(self, default=0)
```

# Upper
```python
Upper(self, default=1e+16)
```

# URDF
```python
URDF(self, mode='model')
```

# URI
```python
URI(self, default='')
```

# UseDynamicMOIRescaling
```python
UseDynamicMOIRescaling(self, default=False)
```

# UseParentModelFrame
```python
UseParentModelFrame(self, default=False)
```

# UsePatchRadius
```python
UsePatchRadius(self, default=True)
```

# Velocity
```python
Velocity(self, default=-1)
```

# VerticalPosition
```python
VerticalPosition(self)
```

# VerticalVelocity
```python
VerticalVelocity(self)
```

# Vertical
```python
Vertical(self)
```

# ViscousFriction
```python
ViscousFriction(self, default=0)
```

# Visual
```python
Visual(self)
```

# Visualize
```python
Visualize(self, default=False)
```

# Waypoint
```python
Waypoint(self)
```

# Width
```python
Width(self, default=320)
```

# World
```python
World(self)
```

# X
```python
X(self)
```

# XYZ
```python
XYZ(self, default=[0, 0, 1])
```

# Y
```python
Y(self)
```

# Z
```python
Z(self)
```


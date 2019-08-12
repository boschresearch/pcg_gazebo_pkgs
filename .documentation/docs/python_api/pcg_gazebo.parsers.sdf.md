
# pcg_gazebo.parsers.sdf


# Accel
```python
Accel()
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
Accuracy()
```


# Actor
```python
Actor()
```


# AllowAutoDisable
```python
AllowAutoDisable()
```


# Altimeter
```python
Altimeter()
```


# AlwaysOn
```python
AlwaysOn()
```


# Ambient
```python
Ambient()
```


# AngularVelocity
```python
AngularVelocity()
```


# Animation
```python
Animation()
```


# Attenuation
```python
Attenuation()
```


# AutoStart
```python
AutoStart()
```


# Axis
```python
Axis()
```


# Axis2
```python
Axis2()
```


# BiasMean
```python
BiasMean()
```


# BiasStdDev
```python
BiasStdDev()
```


# Bounce
```python
Bounce()
```


# Box
```python
Box()
```


# Bullet
```python
Bullet()
```


# Camera
```python
Camera()
```


# CastShadows
```python
CastShadows()
```


# Center
```python
Center()
```


# CFM
```python
CFM()
```


# Child
```python
Child()
```


# Clip
```python
Clip()
```


# Coefficient
```python
Coefficient()
```


# Collision
```python
Collision()
```


# Constant
```python
Constant()
```


# Constraints
```python
Constraints()
```


# ContactMaxCorrectingVel
```python
ContactMaxCorrectingVel()
```


# ContactSurfaceLayer
```python
ContactSurfaceLayer()
```


# Contact
```python
Contact()
```


# Cylinder
```python
Cylinder()
```


# Damping
```python
Damping()
```


# DelayStart
```python
DelayStart()
```


# DepthCamera
```python
DepthCamera()
```


# Diffuse
```python
Diffuse()
```


# Direction
```python
Direction()
```


# Dissipation
```python
Dissipation()
```


# Distortion
```python
Distortion()
```


# DynamicFriction
```python
DynamicFriction()
```


# Dynamics
```python
Dynamics()
```


# Effort
```python
Effort()
```


# Emissive
```python
Emissive()
```


# Empty
```python
Empty()
```


# EnableWind
```python
EnableWind()
```


# ERP
```python
ERP()
```


# FallOff
```python
FallOff()
```


# Far
```python
Far()
```


# FDir1
```python
FDir1()
```

3-tuple specifying direction of mu1 in the collision local reference frame

Args:
    default (list): Direction unit vector

Attributes:
    value (list): Stored direction unit vector


# Filename
```python
Filename()
```


# ForceTorque
```python
ForceTorque()
```


# Format
```python
Format()
```


# Frame
```python
Frame()
```


# FrictionModel
```python
FrictionModel()
```


# Friction
```python
Friction()
```

Configuration of the collision friction parameters.

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction


# Friction2
```python
Friction2()
```


# Geometry
```python
Geometry()
```


# Granularity
```python
Granularity()
```


# Gravity
```python
Gravity()
```


# Height
```python
Height()
```


# HorizontalFOV
```python
HorizontalFOV()
```


# Horizontal
```python
Horizontal()
```


# Image
```python
Image()
```


# IMU
```python
IMU()
```


# Include
```python
Include()
```


# Inertia
```python
Inertia()
```


# Inertial
```python
Inertial()
```


# InitialPosition
```python
InitialPosition()
```


# InnerAngle
```python
InnerAngle()
```


# InterpolateX
```python
InterpolateX()
```


# Iters
```python
Iters()
```


# IXX
```python
IXX()
```


# IXY
```python
IXY()
```


# IXZ
```python
IXZ()
```


# IYY
```python
IYY()
```


# IYZ
```python
IYZ()
```


# IZZ
```python
IZZ()
```


# Joint
```python
Joint()
```


# K1
```python
K1()
```


# K2
```python
K2()
```


# K3
```python
K3()
```


# Kd
```python
Kd()
```


# Kinematic
```python
Kinematic()
```


# Kp
```python
Kp()
```


# LaserRetro
```python
LaserRetro()
```


# Length
```python
Length()
```


# Light
```python
Light()
```


# Lighting
```python
Lighting()
```


# Limit
```python
Limit()
```


# LinearAcceleration
```python
LinearAcceleration()
```


# Linear
```python
Linear()
```


# Link
```python
Link()
```


# Localization
```python
Localization()
```


# Loop
```python
Loop()
```


# Lower
```python
Lower()
```


# Mass
```python
Mass()
```


# Material
```python
Material()
```


# MaxAngle
```python
MaxAngle()
```


# MaxContacts
```python
MaxContacts()
```


# MaxStepSize
```python
MaxStepSize()
```


# MaxTransientVelocity
```python
MaxTransientVelocity()
```


# MaxVel
```python
MaxVel()
```


# Max
```python
Max()
```


# Mean
```python
Mean()
```


# MeasureDirection
```python
MeasureDirection()
```


# Mesh
```python
Mesh()
```


# MinAngle
```python
MinAngle()
```


# MinDepth
```python
MinDepth()
```


# MinStepSize
```python
MinStepSize()
```


# Min
```python
Min()
```


# Model
```python
Model()
```


# Mu
```python
Mu()
```

Coefficient of friction in the range of [0, 1]

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction


# Mu2
```python
Mu2()
```

Second coefficient of friction in the range of [0, 1]

Args:
    default (float): Coefficient of friction

Attributes:
    value (float): Stored coefficient of friction


# MustBeLoopJoint
```python
MustBeLoopJoint()
```


# Name
```python
Name()
```


# Near
```python
Near()
```


# Noise
```python
Noise()
```


# NormalMap
```python
NormalMap()
```


# Normal
```python
Normal()
```


# ODE
```python
ODE()
```


# OrientationReferenceFrame
```python
OrientationReferenceFrame()
```


# OuterAngle
```python
OuterAngle()
```


# Output
```python
Output()
```


# OverrideImpactCaptureVelocity
```python
OverrideImpactCaptureVelocity()
```


# OverrideStictionTransitionVelocity
```python
OverrideStictionTransitionVelocity()
```


# P1
```python
P1()
```


# P2
```python
P2()
```


# Parent
```python
Parent()
```


# PatchRadius
```python
PatchRadius()
```


# Path
```python
Path()
```


# Physics
```python
Physics()
```


# Plane
```python
Plane()
```


# PlasticCoefRestitution
```python
PlasticCoefRestitution()
```


# PlasticImpactVelocity
```python
PlasticImpactVelocity()
```


# Plugin
```python
Plugin()
```


# Point
```python
Point()
```


# Polyline
```python
Polyline()
```


# Pose
```python
Pose()
```


# Precision
```python
Precision()
```


# PreConIters
```python
PreConIters()
```


# ProvideFeedback
```python
ProvideFeedback()
```


# Quadratic
```python
Quadratic()
```


# Radius
```python
Radius()
```


# Range
```python
Range()
```


# Rate
```python
Rate()
```


# Ray
```python
Ray()
```


# RealTimeFactor
```python
RealTimeFactor()
```


# RealTimeUpdateRate
```python
RealTimeUpdateRate()
```


# Resolution
```python
Resolution()
```


# RestitutionCoefficient
```python
RestitutionCoefficient()
```


# RollingFriction
```python
RollingFriction()
```


# Samples
```python
Samples()
```


# Save
```python
Save()
```


# Scale
```python
Scale()
```


# Scan
```python
Scan()
```


# Script
```python
Script()
```


# SDF
```python
SDF()
```


# SelfCollide
```python
SelfCollide()
```


# Sensor
```python
Sensor()
```


# Shader
```python
Shader()
```


# Simbody
```python
Simbody()
```


# Size
```python
Size()
```


# Skin
```python
Skin()
```


# Slip
```python
Slip()
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient


# Slip1
```python
Slip1()
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient


# Slip2
```python
Slip2()
```

Force dependent slip direction 1 in collision local frame, between the
range of [0,1].

Args:
    default (float): Slip coefficient

Attributes:
    value (float): Stored slip coefficient


# SoftCFM
```python
SoftCFM()
```


# SoftERP
```python
SoftERP()
```


# Solver
```python
Solver()
```


# Sor
```python
Sor()
```


# Specular
```python
Specular()
```


# Sphere
```python
Sphere()
```


# SplitImpulsePenetrationThreshold
```python
SplitImpulsePenetrationThreshold()
```


# SplitImpulse
```python
SplitImpulse()
```


# Spot
```python
Spot()
```


# SpringReference
```python
SpringReference()
```


# SpringStiffness
```python
SpringStiffness()
```


# StaticFriction
```python
StaticFriction()
```


# Static
```python
Static()
```


# StdDev
```python
StdDev()
```


# Stiffness
```python
Stiffness()
```


# SubMesh
```python
SubMesh()
```


# SurfaceRadius
```python
SurfaceRadius()
```


# Surface
```python
Surface()
```


# Threshold
```python
Threshold()
```


# Time
```python
Time()
```


# Topic
```python
Topic()
```


# Torsional
```python
Torsional()
```


# Trajectory
```python
Trajectory()
```


# Transparency
```python
Transparency()
```


# Type
```python
Type()
```


# UpdateRate
```python
UpdateRate()
```


# Upper
```python
Upper()
```


# URDF
```python
URDF()
```


# URI
```python
URI()
```


# UseDynamicMOIRescaling
```python
UseDynamicMOIRescaling()
```


# UseParentModelFrame
```python
UseParentModelFrame()
```


# UsePatchRadius
```python
UsePatchRadius()
```


# Velocity
```python
Velocity()
```


# VerticalPosition
```python
VerticalPosition()
```


# VerticalVelocity
```python
VerticalVelocity()
```


# Vertical
```python
Vertical()
```


# ViscousFriction
```python
ViscousFriction()
```


# Visual
```python
Visual()
```


# Visualize
```python
Visualize()
```


# Waypoint
```python
Waypoint()
```


# Width
```python
Width()
```


# World
```python
World()
```


# X
```python
X()
```


# XYZ
```python
XYZ()
```


# Y
```python
Y()
```


# Z
```python
Z()
```


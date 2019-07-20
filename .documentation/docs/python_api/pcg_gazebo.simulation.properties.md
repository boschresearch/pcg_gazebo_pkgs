# pcg_gazebo.simulation.properties

# Axis
```python
Axis(self, xyz=[0, 0, 1], lower_limit=-1e+16, upper_limit=1e+16, velocity_limit=0, effort_limit=0, damping=0, friction=0, spring_reference=0, spring_stiffness=0)
```

# BoundingBox
```python
BoundingBox(self, min_corner=[0, 0, 0], max_corner=[0, 0, 0])
```

# Collision
```python
Collision(self, name='collision', pose=[0, 0, 0, 0, 0, 0], geometry_type=None, geometry_args=None, mu=1.0, mu2=1.0, slip1=0, slip2=0, rolling_friction=1, fdir1=[0, 0, 0], max_contacts=10, soft_cfm=0, soft_erp=0.2, kp=1000000000000.0, kd=1, max_vel=0.01, min_depth=0, split_impulse=1, split_impulse_penetration_threshold=-0.01, enable_friction=False, enable_bounce=False, enable_contact=False)
```

# Footprint
```python
Footprint(self)
```

# Geometry
```python
Geometry(self, geo_type=None, **kwargs)
```

# Inertial
```python
Inertial(self, mass=0, ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
```

# Material
```python
Material(self)
```

# Mesh
```python
Mesh(self, filename=None, load_mesh=False)
```

# Noise
```python
Noise(self, mean=0, stddev=0, bias_mean=0, bias_stddev=0, precision=0, type='none')
```

# Plugin
```python
Plugin(self, name=None, filename=None)
```

# Pose
```python
Pose(self, pos=[0, 0, 0], rpy=None, quat=None)
```

# Visual
```python
Visual(self, name='visual', pose=[0, 0, 0, 0, 0, 0], cast_shadows=True, transparency=0, geometry_type=None, geometry_args=None)
```


# pcg_gazebo.simulation.sensors

# Sensor
```python
Sensor(self, name='sensor', always_on=True, update_rate=50, visualize=False, topic='topic', pose=[0, 0, 0, 0, 0, 0])
```

# Camera
```python
Camera(self, name='camera', always_on=True, update_rate=50, visualize=True, topic='camera', pose=[0, 0, 0, 0, 0, 0], noise_type='gaussian', noise_mean=0, noise_stddev=0, horizontal_fov=1.047, image_width=320, image_height=240, image_format='R8G8B8', clip_near=0.1, clip_far=100, distortion_k1=0, distortion_k2=0, distortion_k3=0, distortion_p1=0, distortion_p2=0, distortion_center=[0.5, 0.5])
```

# Contact
```python
Contact(self, name='contact', always_on=True, update_rate=50, visualize=True, topic='contact', pose=[0, 0, 0, 0, 0, 0], collision_element_name='')
```

# IMU
```python
IMU(self, name='imu', always_on=True, update_rate=50, visualize=False, topic='topic', pose=[0, 0, 0, 0, 0, 0])
```

# Ray
```python
Ray(self, name='ray', always_on=True, update_rate=50, visualize=True, topic='scan', pose=[0, 0, 0, 0, 0, 0], horizontal_samples=640, horizontal_resolution=1, horizontal_min_angle=0, horizontal_max_angle=0, vertical_samples=1, vertical_resolution=1, vertical_min_angle=0, vertical_max_angle=0, range_min=0, range_max=0, range_resolution=0, noise_mean=0, noise_stddev=0)
```


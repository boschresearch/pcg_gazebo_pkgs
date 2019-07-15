

```python
%matplotlib inline
import trimesh
from matplotlib import pyplot as plt
from pcg_gazebo.simulation.properties import Mesh
from pcg_gazebo.simulation import create_object, SimulationModel

```

# Box


```python
# Create model
model = SimulationModel('box')

# Create box link
model.add_cuboid_link('box', mass=0.1, size=[2, 1.2, 3])

print(model.to_sdf('model'))
```


```python
scene = model.create_scene()
#scene.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5, z_limits=[-0.2, 0.2])
plt.show()
```


```python
# Set random orientation and plot footprints again
model.set_random_orientation()
scene = model.create_scene()
#scene.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5, z_limits=[-0.2, 0.2])
plt.show()
```

# Cylinder


```python
# Create model
model = SimulationModel('cylinder')

# Create box link
model.add_cylindrical_link('cylinder', mass=0.1, radius=0.3, length=1)

print(model.to_sdf('model'))
```


```python
scene = model.create_scene()
#scene.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5, z_limits=[-0.2, 0.2])
plt.show()
```


```python
# Set random orientation and plot footprints again
model.set_random_orientation()
scene = model.create_scene()
#scene.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(fig_width=5, fig_height=5, z_limits=[-0.1, 0.1])
plt.show()
```

# Custom mesh


```python
model = SimulationModel.from_gazebo_model('jersey_barrier')
print(model.to_sdf('model'))
```


```python
scene = model.create_scene(mesh_type='collision')
#scene.show()
```


```python
fig = model.plot_footprint(mesh_type='collision', fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(mesh_type='collision', fig_width=5, fig_height=5, z_limits=[0.5, 0.7])
plt.show()
```


```python
# Set random orientation and plot footprints again
model.set_random_orientation()
print(model.pose)
scene = model.create_scene(mesh_type='collision')
#scene.show()
```


```python
fig = model.plot_footprint(mesh_type='collision', fig_width=5, fig_height=5)
plt.show()
```


```python
fig = model.plot_footprint(mesh_type='collision', fig_width=5, fig_height=5, z_limits=[-0.2, 0.2])
plt.show()
```


```python

```

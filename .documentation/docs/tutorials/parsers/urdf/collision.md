
# Collision

The collision properties of a link. Note that this can be different from the visual properties of a link, for example, simpler collision models are often used to reduce computation time. This is a child element of `<link>` and a link can have multiple collisions.


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```


```python
# The collision element is created with an empty geometry by default
collision = create_urdf_element('collision')
print(collision)
```


```python
# Setting the parameters for the visual element

collision.origin.xyz = [1, 0, 1]
collision.origin.rpy = [0.2, 0, 0.6]
print(collision)
```


```python
# Setting different geometries to the visual element
collision.geometry.box = create_urdf_element('box')
print(collision)
```


```python
collision.geometry.sphere = create_urdf_element('sphere')
print(collision)
```


```python
collision.geometry.cylinder = create_urdf_element('cylinder')
print(collision)
```


```python
collision.geometry.mesh = create_urdf_element('mesh')
print(collision)
```


# Visuals

The `<visual>` element specifies the shape of the geometry for rendering. It is a child element from `<link>` and a link can have multiple visual elements.


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```


```python
# The visual element is created with an empty geometry by default
visual = create_urdf_element('visual')
print(visual)
```


```python
# Setting the parameters for the visual element

visual.origin.xyz = [1, 0, 1]
visual.origin.rpy = [0.2, 0, 0.6]
print(visual)
```


```python
# Setting different geometries to the visual element
visual.geometry.box = create_urdf_element('box')
print(visual)
```


```python
visual.geometry.sphere = create_urdf_element('sphere')
print(visual)
```


```python
visual.geometry.cylinder = create_urdf_element('cylinder')
print(visual)
```


```python
visual.geometry.mesh = create_urdf_element('mesh')
print(visual)
```


# Geometries

Geometry entities are child elements of `<visual>` or `<collision>` elements.


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```

## Basic entities
Demonstration of the basic URDF elements that can be generated with and without the optional parameters.

### Geometries

#### Box


```python
# Default box
box = create_urdf_element('box')
print('Default box')
print(box)

print('Default box - as dict')
print(box.to_dict())

print('Default box - as URDF')
print(box.to_xml_as_str())

# Changing the size
box.size = [2, 3, 4]
print('Custom box')
print(box)

# Exporting 
# box.export_xml('/tmp/box.urdf')
```

    Default box
    <box size="0 0 0"/>
    
    Default box - as dict
    {'box': {'attributes': {'size': '0 0 0'}}}
    Default box - as URDF
    <box size="0 0 0"/>
    Custom box
    <box size="2 3 4"/>
    


#### Cylinder


```python
# Default cylinder
cylinder = create_urdf_element('cylinder')
print('Default cylinder')
print(cylinder)

print('Default cylinder - as dict')
print(cylinder.to_dict())

print('Default cylinder - as URDF')
print(cylinder.to_xml_as_str())

# Changing the parameters
cylinder.radius = 2
cylinder.length = 3
print('Custom cylinder')
print(cylinder)

# Exporting 
# cylinder.export_xml('/tmp/cylinder.urdf')
```

    Default cylinder
    <cylinder length="0" radius="0"/>
    
    Default cylinder - as dict
    {'cylinder': {'attributes': {'radius': '0', 'length': '0'}}}
    Default cylinder - as URDF
    <cylinder length="0" radius="0"/>
    Custom cylinder
    <cylinder length="3" radius="2"/>
    


#### Sphere


```python
# Default sphere
sphere = create_urdf_element('sphere')
print('Default sphere')
print(sphere)

print('Default sphere - as dict')
print(sphere.to_dict())

print('Default sphere - as URDF')
print(sphere.to_xml_as_str())

# Changing the parameters
sphere.radius = 2
print('Custom sphere')
print(sphere)

# Exporting 
# sphere.export_xml('/tmp/sphere.urdf')
```

    Default sphere
    <sphere radius="0"/>
    
    Default sphere - as dict
    {'sphere': {'attributes': {'radius': '0'}}}
    Default sphere - as URDF
    <sphere radius="0"/>
    Custom sphere
    <sphere radius="2"/>
    


#### Mesh


```python
mesh = create_urdf_element('mesh')
print('Default mesh - with default parameters')
print(mesh)

print('Default mesh - as dict')
print(mesh.to_dict())

print('Default mesh - as URDF')
print(mesh.to_xml_as_str())

# Changing the parameters
mesh.filename = 'package://mesh.dae'
print('Custom mesh')
print(mesh)

# Exporting 
# mesh.export_xml('/tmp/mesh.urdf')
```

    Default mesh - with default parameters
    <mesh filename=""/>
    
    Default mesh - as dict
    {'mesh': {'attributes': {'filename': ''}}}
    Default mesh - as URDF
    <mesh filename=""/>
    Custom mesh
    <mesh filename="package://mesh.dae"/>
    


### Creating a geometry entity



```python
# Initially, the geometry is created with a <box/> element
geometry = create_urdf_element('geometry')
print(geometry)
```

    <geometry>
      <box size="0 0 0"/>
    </geometry>
    



```python
# Creating a geometry for each of the basic forms
# The geometry entity has a separate mode for each possible geometry forms, since it cannot hold
# multiple geometries
# When a new geometry is set, the former one is deleted
print('All the geometry entity modes:')
print(geometry.get_modes())

geometry.box = box
print(geometry)

geometry.cylinder = cylinder
print(geometry)

geometry.sphere = sphere
print(geometry)

geometry.mesh = mesh
print(geometry)
```

    All the geometry entity modes:
    ['box', 'cylinder', 'sphere', 'mesh']
    <geometry>
      <box size="2 3 4"/>
    </geometry>
    
    <geometry>
      <cylinder length="3" radius="2"/>
    </geometry>
    
    <geometry>
      <sphere radius="2"/>
    </geometry>
    
    <geometry>
      <mesh filename="package://mesh.dae"/>
    </geometry>
    



```python

```

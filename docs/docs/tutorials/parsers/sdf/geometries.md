
# Geometries

Geometry entities are child elements of `<visual>` or `<collision>` elements.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element

```

## Basic entities
Demonstration of the basic SDF elements that can be generated with and without the optional parameters.

### Geometries

#### Box


```python
# Default box
box = create_sdf_element('box')
print('Default box')
print(box)

print('Default box - as dict')
print(box.to_dict())

print('Default box - as SDF')
print(box.to_xml_as_str())

# Changing the size
box.size = [2, 3, 4]
print('Custom box')
print(box)

# Exporting 
# box.export_xml('/tmp/box.sdf')
```

    Default box
    <box>
      <size>0 0 0</size>
    </box>
    
    Default box - as dict
    {'box': {'size': {'value': [0, 0, 0]}}}
    Default box - as SDF
    <box><size>0 0 0</size></box>
    Custom box
    <box>
      <size>2 3 4</size>
    </box>
    


#### Cylinder


```python
# Default cylinder
cylinder = create_sdf_element('cylinder')
print('Default cylinder')
print(cylinder)

print('Default cylinder - as dict')
print(cylinder.to_dict())

print('Default cylinder - as SDF')
print(cylinder.to_xml_as_str())

# Changing the parameters
cylinder.radius = 2
cylinder.length = 3
print('Custom cylinder')
print(cylinder)

# Exporting 
# cylinder.export_xml('/tmp/cylinder.sdf')
```

    Default cylinder
    <cylinder>
      <length>0</length>
      <radius>0</radius>
    </cylinder>
    
    Default cylinder - as dict
    {'cylinder': {'length': {'value': 0}, 'radius': {'value': 0}}}
    Default cylinder - as SDF
    <cylinder><length>0</length><radius>0</radius></cylinder>
    Custom cylinder
    <cylinder>
      <length>3.0</length>
      <radius>2.0</radius>
    </cylinder>
    


#### Sphere


```python
# Default sphere
sphere = create_sdf_element('sphere')
print('Default sphere')
print(sphere)

print('Default sphere - as dict')
print(sphere.to_dict())

print('Default sphere - as SDF')
print(sphere.to_xml_as_str())

# Changing the parameters
sphere.radius = 2
print('Custom sphere')
print(sphere)

# Exporting 
# sphere.export_xml('/tmp/sphere.sdf')
```

    Default sphere
    <sphere>
      <radius>0</radius>
    </sphere>
    
    Default sphere - as dict
    {'sphere': {'radius': {'value': 0}}}
    Default sphere - as SDF
    <sphere><radius>0</radius></sphere>
    Custom sphere
    <sphere>
      <radius>2.0</radius>
    </sphere>
    


#### Plane


```python
# Default plane
plane = create_sdf_element('plane')
print('Default plane')
print(plane)

print('Default plane - as dict')
print(plane.to_dict())

print('Default plane - as SDF')
print(plane.to_xml_as_str())

# Changing the parameters
# Length of each side of the plane
plane.size = [10, 10]
# Normal direction of the plane
plane.normal = [1, 0, 0]
print('Custom plane')
print(plane)

# Exporting 
# plane.export_xml('/tmp/plane.sdf')
```

    Default plane
    <plane>
      <size>0 0</size>
      <normal>0 0 1</normal>
    </plane>
    
    Default plane - as dict
    {'plane': {'size': {'value': [0, 0]}, 'normal': {'value': [0, 0, 1]}}}
    Default plane - as SDF
    <plane><size>0 0</size><normal>0 0 1</normal></plane>
    Custom plane
    <plane>
      <size>10 10</size>
      <normal>1 0 0</normal>
    </plane>
    


#### Image

A grayscale image can be used to extrude a set of boxes


```python
# Default image
image = create_sdf_element('image')
print('Default image')
print(image)

print('Default image - as dict')
print(image.to_dict())

print('Default image - as SDF')
print(image.to_xml_as_str())

# Height of the extruded boxes
image.height = 10
# The amount of error in the model
image.granularity = 1
# Grayscale threshold
image.threshold = 100
# Scaling factor applied to the image
image.scale = [2]
# URI of the grayscale image
image.uri = 'filename'

print('Custom image')
print(image)

# Exporting 
# image.export_xml('/tmp/image.sdf')
```

    Default image
    <image>
      <granularity>1</granularity>
      <height>1</height>
      <scale>1</scale>
      <uri></uri>
      <threshold>0</threshold>
    </image>
    
    Default image - as dict
    {'image': {'granularity': {'value': 1}, 'height': {'value': 1}, 'scale': {'value': [1]}, 'uri': {'value': ''}, 'threshold': {'value': 0}}}
    Default image - as SDF
    <image><granularity>1</granularity><height>1</height><scale>1</scale><uri></uri><threshold>0</threshold></image>
    Custom image
    <image>
      <granularity>1</granularity>
      <height>10.0</height>
      <scale>2</scale>
      <uri>filename</uri>
      <threshold>100.0</threshold>
    </image>
    


#### Mesh


```python
mesh = create_sdf_element('mesh')
print('Default mesh - with default parameters')
print(mesh)

print('Default mesh - as dict')
print(mesh.to_dict())

print('Default mesh - as SDF')
print(mesh.to_xml_as_str())
```

    Default mesh - with default parameters
    <mesh>
      <scale>1 1 1</scale>
      <uri></uri>
    </mesh>
    
    Default mesh - as dict
    {'mesh': {'scale': {'value': [1, 1, 1]}, 'uri': {'value': ''}}}
    Default mesh - as SDF
    <mesh><scale>1 1 1</scale><uri></uri></mesh>



```python
print('Mesh with optional parameters')
mesh.reset(with_optional_elements=True)
print(mesh)

# Name of the submesh under the parent mesh
mesh.submesh.name = 'submesh.stl'
# Set to true to center the vertices of the submesh at (0, 0, 0)
mesh.submesh.center = True
# Scaling factor of the mesh
mesh.scale = [2, 1, 1]
# URI of the mesh
mesh.uri = 'file://mesh.stl'

print('Custom mesh')
print(mesh)
```

    Mesh with optional parameters
    <mesh>
      <scale>1 1 1</scale>
      <uri></uri>
      <submesh>
        <center>0</center>
        <name>none</name>
      </submesh>
    </mesh>
    
    Custom mesh
    <mesh>
      <scale>2 1 1</scale>
      <uri>file://mesh.stl</uri>
      <submesh>
        <center>1</center>
        <name>submesh.stl</name>
      </submesh>
    </mesh>
    


#### Polyline


```python
polyline = create_sdf_element('polyline')
print('Default polyline - with default parameters')
print(polyline)

print('Default polyline - as dict')
print(polyline.to_dict())

print('Default polyline - as SDF')
print(polyline.to_xml_as_str())
```

    Default polyline - with default parameters
    <polyline>
      <height>1</height>
    </polyline>
    
    Default polyline - as dict
    {'polyline': {'height': {'value': 1}}}
    Default polyline - as SDF
    <polyline><height>1</height></polyline>



```python
print('Polyline with optional parameters')
polyline.reset(with_optional_elements=True)
print(polyline)

# Set new height
polyline.height = 2.3
# Customize point
polyline.points[0].value = [2.3, 4.5]
# Add new point
polyline.add_point()
# Set the coordinates of new point
polyline.points[1].value = [3.7, 10.1]

print('Custom polyline')
print(polyline)
```

    Polyline with optional parameters
    <polyline>
      <point>0 0</point>
      <height>1</height>
    </polyline>
    
    Custom polyline
    <polyline>
      <point>2.3 4.5</point>
      <point>3.7 10.1</point>
      <height>2.3</height>
    </polyline>
    


### Creating a geometry entity



```python
# Initially, the geometry is created with a <empty/> element
geometry = create_sdf_element('geometry')
print(geometry)
```

    <geometry>
      <empty></empty>
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

geometry.plane = plane
print(geometry)

geometry.image = image
print(geometry)

geometry.mesh = mesh
print(geometry)

geometry.polyline = polyline
print(geometry)
```

    All the geometry entity modes:
    ['empty', 'box', 'image', 'cylinder', 'sphere', 'plane', 'mesh', 'polyline']
    <geometry>
      <box>
        <size>2 3 4</size>
      </box>
    </geometry>
    
    <geometry>
      <cylinder>
        <length>3.0</length>
        <radius>2.0</radius>
      </cylinder>
    </geometry>
    
    <geometry>
      <sphere>
        <radius>2.0</radius>
      </sphere>
    </geometry>
    
    <geometry>
      <plane>
        <size>10 10</size>
        <normal>1 0 0</normal>
      </plane>
    </geometry>
    
    <geometry>
      <image>
        <granularity>1</granularity>
        <height>10.0</height>
        <scale>2</scale>
        <uri>filename</uri>
        <threshold>100.0</threshold>
      </image>
    </geometry>
    
    <geometry>
      <mesh>
        <scale>2 1 1</scale>
        <uri>file://mesh.stl</uri>
        <submesh>
          <center>1</center>
          <name>submesh.stl</name>
        </submesh>
      </mesh>
    </geometry>
    
    <geometry>
      <polyline>
        <height>2.3</height>
        <point>2.3 4.5</point>
        <point>3.7 10.1</point>
      </polyline>
    </geometry>
    


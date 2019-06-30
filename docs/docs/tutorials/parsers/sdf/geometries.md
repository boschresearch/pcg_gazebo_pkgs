
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
      <radius>0</radius>
      <length>0</length>
    </cylinder>
    
    Default cylinder - as dict
    {'cylinder': {'radius': {'value': 0}, 'length': {'value': 0}}}
    Default cylinder - as SDF
    <cylinder><radius>0</radius><length>0</length></cylinder>
    Custom cylinder
    <cylinder>
      <radius>2.0</radius>
      <length>3.0</length>
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
      <normal>0 0 1</normal>
      <size>0 0</size>
    </plane>
    
    Default plane - as dict
    {'plane': {'normal': {'value': [0, 0, 1]}, 'size': {'value': [0, 0]}}}
    Default plane - as SDF
    <plane><normal>0 0 1</normal><size>0 0</size></plane>
    Custom plane
    <plane>
      <normal>1 0 0</normal>
      <size>10 10</size>
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
      <height>1</height>
      <uri></uri>
      <granularity>1</granularity>
      <threshold>0</threshold>
      <scale>1</scale>
    </image>
    
    Default image - as dict
    {'image': {'height': {'value': 1}, 'uri': {'value': ''}, 'threshold': {'value': 0}, 'granularity': {'value': 1}, 'scale': {'value': [1]}}}
    Default image - as SDF
    <image><height>1</height><uri></uri><granularity>1</granularity><threshold>0</threshold><scale>1</scale></image>
    Custom image
    <image>
      <height>10.0</height>
      <uri>filename</uri>
      <granularity>1</granularity>
      <threshold>100.0</threshold>
      <scale>2</scale>
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
      <uri></uri>
      <scale>1 1 1</scale>
    </mesh>
    
    Default mesh - as dict
    {'mesh': {'uri': {'value': ''}, 'scale': {'value': [1, 1, 1]}}}
    Default mesh - as SDF
    <mesh><uri></uri><scale>1 1 1</scale></mesh>



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
      <uri></uri>
      <submesh>
        <name>none</name>
        <center>0</center>
      </submesh>
      <scale>1 1 1</scale>
    </mesh>
    
    Custom mesh
    <mesh>
      <uri>file://mesh.stl</uri>
      <submesh>
        <name>submesh.stl</name>
        <center>1</center>
      </submesh>
      <scale>2 1 1</scale>
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
      <height>1</height>
      <point>0 0</point>
    </polyline>
    
    Custom polyline
    <polyline>
      <height>2.3</height>
      <point>2.3 4.5</point>
      <point>3.7 10.1</point>
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
        <radius>2.0</radius>
        <length>3.0</length>
      </cylinder>
    </geometry>
    
    <geometry>
      <sphere>
        <radius>2.0</radius>
      </sphere>
    </geometry>
    
    <geometry>
      <plane>
        <normal>1 0 0</normal>
        <size>10 10</size>
      </plane>
    </geometry>
    
    <geometry>
      <image>
        <height>10.0</height>
        <uri>filename</uri>
        <threshold>100.0</threshold>
        <granularity>1</granularity>
        <scale>2</scale>
      </image>
    </geometry>
    
    <geometry>
      <mesh>
        <uri>file://mesh.stl</uri>
        <submesh>
          <name>submesh.stl</name>
          <center>1</center>
        </submesh>
        <scale>2 1 1</scale>
      </mesh>
    </geometry>
    
    <geometry>
      <polyline>
        <height>2.3</height>
        <point>2.3 4.5</point>
        <point>3.7 10.1</point>
      </polyline>
    </geometry>
    


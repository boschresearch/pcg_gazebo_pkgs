
# Visuals

The `<visual>` element specifies the shape of the geometry for rendering. It is a child element from `<link>` and a link can have multiple visual elements.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# The visual element is created with an empty geometry by default
visual = create_sdf_element('visual')
print(visual)
```

    <visual name="visual">
      <geometry>
        <empty></empty>
      </geometry>
    </visual>
    



```python
# To see the optional elements, use the method reset()
visual.reset(with_optional_elements=True)
print(visual)
```

    <visual name="visual">
      <pose frame="">0 0 0 0 0 0</pose>
      <geometry>
        <empty></empty>
      </geometry>
      <transparency>0</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    



```python
# Setting the parameters for the visual element

# Each visual in a link should have an unique name
visual.name = 'custom_visual'
# If cast_shadows is true, the geometry will cast shadows
visual.cast_shadows = True
# The transparency is a double in the range of [0, 1], 0 being opaque and 1 fully transparent
visual.transparency = 0.2
# Pose of the visual geometry with respect to a frame
visual.pose = [0, 0.2, 0, 0, 0, 0]
visual.pose.frame = 'base_link'

print(visual)
```

    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <empty></empty>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    



```python
# Setting different geometries to the visual element
visual.geometry.box = create_sdf_element('box')
print(visual)
visual.geometry.sphere = create_sdf_element('sphere')
print(visual)
visual.geometry.cylinder = create_sdf_element('cylinder')
print(visual)
visual.geometry.plane = create_sdf_element('plane')
print(visual)
visual.geometry.mesh = create_sdf_element('mesh')
visual.geometry.mesh.reset(with_optional_elements=True)
print(visual)
visual.geometry.image = create_sdf_element('image')
print(visual)
visual.geometry.polyline = create_sdf_element('polyline')
print(visual)
```

    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <sphere>
          <radius>0</radius>
        </sphere>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <cylinder>
          <radius>0</radius>
          <length>0</length>
        </cylinder>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <plane>
          <size>0 0</size>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <mesh>
          <uri></uri>
          <submesh>
            <name>none</name>
            <center>0</center>
          </submesh>
          <scale>1 1 1</scale>
        </mesh>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <image>
          <uri></uri>
          <height>1</height>
          <granularity>1</granularity>
          <threshold>0</threshold>
          <scale>1</scale>
        </image>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    
    <visual name="custom_visual">
      <pose frame="base_link">0 0.2 0 0 0 0</pose>
      <geometry>
        <polyline>
          <height>1</height>
        </polyline>
      </geometry>
      <transparency>0.2</transparency>
      <cast_shadows>1</cast_shadows>
      <material>
        <specular>0.1 0.1 0.1 1</specular>
        <diffuse>0 0 0 1</diffuse>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <emissive>0 0 0 1</emissive>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <lighting>0</lighting>
        <ambient>0 0 0 1</ambient>
      </material>
    </visual>
    



```python
# Optional elements can also be created dynamically
visual = create_sdf_element('visual')
print(visual)
```

    <visual name="visual">
      <geometry>
        <empty></empty>
      </geometry>
    </visual>
    



```python
visual.cast_shadows = True
print(visual)

```

    <visual name="visual">
      <geometry>
        <empty></empty>
      </geometry>
      <cast_shadows>1</cast_shadows>
    </visual>
    



```python
visual.pose = [0, 0.2, 0, 0, 0, 0]
print(visual)
```

    <visual name="visual">
      <pose frame="">0 0.2 0 0 0 0</pose>
      <geometry>
        <empty></empty>
      </geometry>
      <cast_shadows>1</cast_shadows>
    </visual>
    



```python
# The geometry entity can be set with a dictionary with all the child parameters
visual.geometry.box = dict(size=[2, 3, 4])
print(visual)
```

    <visual name="visual">
      <pose frame="">0 0.2 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 3 4</size>
        </box>
      </geometry>
      <cast_shadows>1</cast_shadows>
    </visual>
    



```python
# The pose, as other variables, can be set using a dictionary
# For SDF elements with no child elements, only values, the dictionary must always have a key 'value'
#      d = {value=[0, 0, 0, 0, 0, 0]}
# If the element contains attributes, as the attribute 'frame' in the element 'pose', there should be a key
# 'attributes' with a dictionary containing all the attributes
#      d = {value=[0, 0, 0, 0, 0, 0], attributes=dict(frame='new_frame')}
visual.pose = {'value': [0, 0.2, 0, 0, 0, 0], 'attributes': {'frame': 'new_frame'}}
print(visual)
```

    <visual name="visual">
      <pose frame="new_frame">0 0.2 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 3 4</size>
        </box>
      </geometry>
      <cast_shadows>1</cast_shadows>
    </visual>
    


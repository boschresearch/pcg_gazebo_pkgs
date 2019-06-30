
# Materials

Child element of a `<visual>` element. A visual may contain only one material.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# Material elements are initially empty since all its child elements are optional
material = create_sdf_element('material')
print(material)
```

    <material/>
    



```python
# To create all optional elements with its default elements, use reset()
material.reset(with_optional_elements=True)
print(material)
```

    <material>
      <specular>0.1 0.1 0.1 1</specular>
      <shader type="pixel">
        <normal_map>default</normal_map>
      </shader>
      <diffuse>0 0 0 1</diffuse>
      <script>
        <name>default</name>
        <uri>file://media/materials/scripts/gazebo.material</uri>
      </script>
      <ambient>0 0 0 1</ambient>
      <lighting>0</lighting>
      <emissive>0 0 0 1</emissive>
    </material>
    


## Creating a custom material


```python
# Setting the script
# URI of the material script file
# Gazebo's default material file can be found at
# https://bitbucket.org/osrf/gazebo/src/bd1cdea8fa3d71d6afcdaa83c9b01891d5f72e71/media/materials/scripts/gazebo.material?at=default&fileviewer=file-view-default
material.script.uri = 'file://media/materials/scripts/gazebo.material'
# Name of the script within the script file
material.script.name = 'Gazebo/Wood'

# Setting the shader
# The shader type can be one of the list below:
# ['vertex', 'pixel', 'normal_map_objectspace', 'normal_map_tangentspace']
material.shader.type = 'vertex'
material.shader.normal_map = 'file://normal_map'

# If lighting is false, dynamic lighting will be disabled
material.lighting = False
# The colors must be a set of four numbers in the range of [0, 1]
material.ambient = [0.2, 0.2, 0.2, 1]
material.diffuse = [0.2, 0.4, 0.2, 1]
material.specular = [0.2, 0.2, 0.6, 1]
material.emissive = [0.5, 0.5, 0.5, 1]

print(material)
```

    <material>
      <specular>0.2 0.2 0.6 1</specular>
      <shader type="vertex">
        <normal_map>file://normal_map</normal_map>
      </shader>
      <diffuse>0.2 0.4 0.2 1</diffuse>
      <script>
        <name>Gazebo/Wood</name>
        <uri>file://media/materials/scripts/gazebo.material</uri>
      </script>
      <ambient>0.2 0.2 0.2 1</ambient>
      <lighting>0</lighting>
      <emissive>0.5 0.5 0.5 1</emissive>
    </material>
    


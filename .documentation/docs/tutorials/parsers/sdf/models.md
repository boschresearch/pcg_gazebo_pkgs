
# Models

A model element defines a complete robot or any other kind of physical object.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# The model is empty by default
model = create_sdf_element('model')
print(model)
```

    <model name="model"/>
    



```python
# Let's create the elements dynamically at first
model = create_sdf_element('model')

# The model's name must be unique in a model
model.name = 'my_robot'
print(model)
```

    <model name="my_robot"/>
    



```python
# If static is set to true, the model is immovable
model.static = True
print(model)
```

    <model name="my_robot">
      <static>1</static>
    </model>
    



```python
# With the allow_auto_disable flag on, the physics engine can skip
# updating the model when the model is at rest
# This is only used for models with no joints
model.allow_auto_disable = True
print(model)
```

    <model name="my_robot">
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
    </model>
    



```python
# If self_collide is set to true, the model will collide with
# others expect with those connected by a joint
model.self_collide = True
print(model)
```

    <model name="my_robot">
      <static>1</static>
      <self_collide>1</self_collide>
      <allow_auto_disable>1</allow_auto_disable>
    </model>
    



```python
# Pose of model with respect to the world frame
model.pose = [0, 1, 1, 0, 0, 0]
print(model)
```

    <model name="my_robot">
      <static>1</static>
      <self_collide>1</self_collide>
      <allow_auto_disable>1</allow_auto_disable>
      <pose frame="">0 1 1 0 0 0</pose>
    </model>
    



```python
# An empty link can be added by using the add_link() method as follows
model.add_link(name='link_1')
print(model.links[0])
```

    <link name="link_1"/>
    



```python
# The link can be edited as well
model.links[0].gravity = True
print(model)
```

    <model name="my_robot">
      <static>1</static>
      <self_collide>1</self_collide>
      <allow_auto_disable>1</allow_auto_disable>
      <pose frame="">0 1 1 0 0 0</pose>
      <link name="link_1">
        <gravity>1</gravity>
      </link>
    </model>
    



```python
# Another link with the same name cannot be added
model.add_link('link_1')

```

    Link element with name link_1 already exists



```python
# A model from another file can be included using the add_include()
# method
model.add_include('include_1')
model.includes[0].uri = 'file://some_file'
model.includes[0].name = 'some_model'
print(model)
```

    <model name="my_robot">
      <link name="link_1">
        <gravity>1</gravity>
      </link>
      <pose frame="">0 1 1 0 0 0</pose>
      <include>
        <name>some_model</name>
        <uri>file://some_file</uri>
      </include>
      <static>1</static>
      <self_collide>1</self_collide>
      <allow_auto_disable>1</allow_auto_disable>
    </model>
    



```python
# An empty link can be added by using the add_link() method as follows
model.add_joint(name='joint_1')
print(model.joints[0])
```

    <joint name="joint_1" type="revolute">
      <child>none</child>
      <parent>parent</parent>
    </joint>
    


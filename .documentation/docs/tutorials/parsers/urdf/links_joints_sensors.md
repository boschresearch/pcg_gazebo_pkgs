
# Links, joints and sensors

## Links

A physical link in the simulation contains inertia, collision and visual properties. A link must be a child of a robot and a robot can have multiple links.


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```


```python
# The link is empty by default
link = create_urdf_element('link')
print(link)
```


```python
# By using reset(), it is possible to see the optional elements of a link
link.reset(with_optional_elements=True)
print(link)
```


```python
# Let's create the elements dynamically at first
link = create_urdf_element('link')

# The link's name must be unique in a model
link.name = 'base_link'
print(link)
```


```python
# Mass of the link in kg
link.mass = 30
# The center of mass are the cartesian coordinates in link.inertial.pose
link.origin = create_urdf_element('origin')
link.origin.xyz = [0, 10, 0]
# The moments of inertia describe the elements of the 3x3 rotational inertial matrix
link.inertia.ixx = 0.5
link.inertia.iyy = 0.5
link.inertia.izz = 0.5
print(link)
```

## Joints


```python
# The joint is empty by default
joint = create_urdf_element('joint')
print(joint)
```


```python
# By using reset(), it is possible to see the optional elements of a joint
joint.reset(with_optional_elements=True)
print(joint)
```


# Robots


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```


```python
robot = create_urdf_element('robot')
print(robot)
```


```python
robot.reset(with_optional_elements=True)
print(robot)
```

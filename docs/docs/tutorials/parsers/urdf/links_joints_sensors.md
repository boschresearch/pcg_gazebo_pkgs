
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

    <link name="link"/>
    



```python
# By using reset(), it is possible to see the optional elements of a link
link.reset(with_optional_elements=True)
print(link)
```

    <link name="link">
      <inertial>
        <mass value="0"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </inertial>
      <visual name="visual">
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <gazebo/>
        <material name="">
          <gazebo/>
          <color rgba="0 0 0 1"/>
        </material>
        <geometry>
          <box size="0 0 0"/>
        </geometry>
      </visual>
      <gazebo>
        <mu2>0</mu2>
        <minDepth>0</minDepth>
        <mu1>0</mu1>
        <selfCollide>0</selfCollide>
        <kd>1</kd>
        <kp>1000000000000.0</kp>
        <maxVel>0.01</maxVel>
        <maxContacts>20</maxContacts>
      </gazebo>
      <collision name="collision">
        <gazebo/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <box size="0 0 0"/>
        </geometry>
      </collision>
    </link>
    



```python
# Let's create the elements dynamically at first
link = create_urdf_element('link')

# The link's name must be unique in a model
link.name = 'base_link'
print(link)
```

    <link name="base_link"/>
    



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

    <link name="base_link">
      <inertial>
        <mass value="0"/>
        <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </inertial>
    </link>
    


## Joints


```python
# The joint is empty by default
joint = create_urdf_element('joint')
print(joint)
```

    <joint name="joint" type="revolute">
      <limit effort="0" lower="0" upper="0" velocity="0"/>
      <child link="link"/>
      <parent link="link"/>
    </joint>
    



```python
# By using reset(), it is possible to see the optional elements of a joint
joint.reset(with_optional_elements=True)
print(joint)
```

    <joint name="joint" type="revolute">
      <mimic multiplier="1" offset="0"/>
      <safety_controller k_position="0" k_velocity="0" soft_lower_limit="0" soft_upper_limit="0"/>
      <gazebo>
        <stopErp>0.2</stopErp>
        <stopCfm>0.0</stopCfm>
      </gazebo>
      <limit effort="0" lower="0" upper="0" velocity="0">
        <gazebo/>
      </limit>
      <axis xyz="1 0 0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <dynamics damping="0" friction="0">
        <gazebo/>
      </dynamics>
      <child link="link"/>
      <parent link="link"/>
    </joint>
    



# Robots


```python
# Import the element creator
from pcg_gazebo.parsers.urdf import create_urdf_element
```


```python
robot = create_urdf_element('robot')
print(robot)
```

    <robot name="robot"/>
    



```python
robot.reset(with_optional_elements=True)
print(robot)
```

    <robot name="robot">
      <joint name="joint" type="revolute">
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <dynamics damping="0" friction="0">
          <gazebo/>
        </dynamics>
        <child link="link"/>
        <parent link="link"/>
        <limit effort="0" lower="0" upper="0" velocity="0">
          <gazebo/>
        </limit>
        <gazebo>
          <stopCfm>0.0</stopCfm>
          <stopErp>0.2</stopErp>
        </gazebo>
        <safety_controller k_position="0" k_velocity="0" soft_lower_limit="0" soft_upper_limit="0"/>
        <mimic multiplier="1" offset="0"/>
        <axis xyz="1 0 0"/>
      </joint>
      <transmission name="transmission">
        <joint name="joint">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="actuator">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </actuator>
        <type>transmission_interface/SimpleTransmission</type>
      </transmission>
      <link name="link">
        <visual name="visual">
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <geometry>
            <box size="0 0 0"/>
          </geometry>
          <material name="">
            <color rgba="0 0 0 1"/>
            <gazebo/>
          </material>
          <gazebo/>
        </visual>
        <collision name="collision">
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <geometry>
            <box size="0 0 0"/>
          </geometry>
          <gazebo/>
        </collision>
        <inertial>
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
          <mass value="0"/>
        </inertial>
        <gazebo>
          <minDepth>0</minDepth>
          <mu2>0</mu2>
          <maxContacts>20</maxContacts>
          <maxVel>0.01</maxVel>
          <kp>1000000000000.0</kp>
          <kd>1</kd>
          <mu1>0</mu1>
          <selfCollide>0</selfCollide>
        </gazebo>
      </link>
      <gazebo/>
    </robot>
    


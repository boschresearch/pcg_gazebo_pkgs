
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
      <link name="link">
        <visual name="visual">
          <material name="">
            <color rgba="0 0 0 1"/>
            <gazebo/>
          </material>
          <geometry>
            <box size="0 0 0"/>
          </geometry>
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <gazebo/>
        </visual>
        <collision name="collision">
          <geometry>
            <box size="0 0 0"/>
          </geometry>
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <gazebo/>
        </collision>
        <inertial>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <mass value="0"/>
        </inertial>
        <gazebo>
          <maxVel>0.01</maxVel>
          <kd>1</kd>
          <maxContacts>20</maxContacts>
          <mu2>0</mu2>
          <selfCollide>0</selfCollide>
          <mu1>0</mu1>
          <minDepth>0</minDepth>
          <kp>1000000000000.0</kp>
        </gazebo>
      </link>
      <joint name="joint" type="revolute">
        <axis xyz="1 0 0"/>
        <parent link="link"/>
        <gazebo>
          <stopErp>0.2</stopErp>
          <stopCfm>0.0</stopCfm>
        </gazebo>
        <limit effort="0" lower="0" upper="0" velocity="0">
          <gazebo/>
        </limit>
        <mimic multiplier="1" offset="0"/>
        <safety_controller k_position="0" k_velocity="0" soft_lower_limit="0" soft_upper_limit="0"/>
        <child link="link"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <dynamics damping="0" friction="0">
          <gazebo/>
        </dynamics>
      </joint>
      <transmission name="transmission">
        <actuator name="actuator">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </actuator>
        <joint name="joint">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <type>transmission_interface/SimpleTransmission</type>
      </transmission>
      <gazebo/>
    </robot>
    


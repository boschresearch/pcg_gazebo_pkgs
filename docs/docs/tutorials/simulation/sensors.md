
# Sensors


```python
from pcg_gazebo.simulation import create_object, SimulationModel
from pcg_gazebo.task_manager import get_rostopic_list
```


```python
# If there is a Gazebo instance running, you can spawn the box into the simulation
from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('default', ros_port=11311, gazebo_port=11345)
simulation = server.get_simulation('default')
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with all default parameters
simulation.create_gazebo_empty_world_task()
simulation.create_rqt_task()

# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()
```

    ['rqt', 'gazebo']
    Is Gazebo running: False



```python
from pcg_gazebo.generators import WorldGenerator
import random
# Create a Gazebo proxy
gazebo_proxy = simulation.get_gazebo_proxy()

# Use the generator to spawn the model to the Gazebo instance running at the moment
generator = WorldGenerator(gazebo_proxy=gazebo_proxy)
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
```

    Is Gazebo running: True


## Sensors

### Standalone IMU sensor


```python
imu_model = SimulationModel(name='default_imu')

# Default IMU sensor
imu_model.add_imu_sensor(
    add_visual=True, 
    add_collision=True, 
    visualize=True,
    mass=0.01,
    size=[0.1, 0.1, 0.1],
    topic='/imu',
    link_shape='cuboid',
    link_name='imu_link')

print(imu_model.to_sdf())

# Spawn IMU standalone model
generator.spawn_model(
    model=imu_model, 
    robot_namespace='default_imu',
    pos=[0, 0, 0.05])
```

    <model name="default_imu">
      <link name="imu_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <transparency>0.0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <sensor name="imu" type="imu">
          <always_on>1</always_on>
          <imu>
            <angular_velocity>
              <y>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </y>
              <x>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </x>
              <z>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </z>
            </angular_velocity>
            <linear_acceleration>
              <y>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </y>
              <x>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </x>
              <z>
                <noise type="none">
                  <stddev>0.0</stddev>
                  <bias_mean>0.0</bias_mean>
                  <bias_stddev>0.0</bias_stddev>
                  <precision>0.0</precision>
                  <mean>0.0</mean>
                </noise>
              </z>
            </linear_acceleration>
          </imu>
          <visualize>1</visualize>
          <topic>/imu</topic>
          <update_rate>50.0</update_rate>
          <plugin filename="libgazebo_ros_imu_sensor.so" name="imu">
            <updateRateHZ>50</updateRateHZ>
            <topicName>/imu</topicName>
            <bodyName>imu_link</bodyName>
            <frameName>world</frameName>
            <alwaysOn>1</alwaysOn>
            <robotNamespace></robotNamespace>
            <gaussianNoise>0</gaussianNoise>
          </plugin>
        </sensor>
        <inertial>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <mass>0.01</mass>
          <inertia>
            <izz>1.6666666666666667e-05</izz>
            <ixx>1.6666666666666667e-05</ixx>
            <ixy>0.0</ixy>
            <iyz>0.0</iyz>
            <iyy>1.6666666666666667e-05</iyy>
            <ixz>0.0</ixz>
          </inertia>
        </inertial>
        <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      </link>
      <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      <allow_auto_disable>0</allow_auto_disable>
      <self_collide>0</self_collide>
      <static>0</static>
    </model>
    





    True




```python
print('List of ROS topics:')
for topic in simulation.get_rostopic_list():
    print(' - ' + topic)
```

    List of ROS topics:
     - /clock
     - /gazebo/link_states
     - /gazebo/model_states
     - /gazebo/parameter_descriptions
     - /gazebo/parameter_updates
     - /gazebo/set_link_state
     - /gazebo/set_model_state
     - /gazebo_gui/parameter_descriptions
     - /gazebo_gui/parameter_updates
     - /imu
     - /rosout
     - /rosout_agg
     - /tf
     - /tf_static


### Standalone ray sensor


```python
ray_model = SimulationModel(name='default_ray')

# Default ray sensor
ray_model.add_ray_sensor(
    add_visual=True, 
    add_collision=True, 
    add_ros_plugin=False,
    mass=0.01,
    radius=0.05,
    link_shape='spherical',    
    link_name='ray_link')

print(ray_model.to_sdf())

# Spawn ray sensor standalone model
generator.spawn_model(
    model=ray_model, 
    robot_namespace='default_ray',
    pos=[0, 0.3, 0.05])
```

    <model name="default_ray">
      <link name="ray_link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <transparency>0.0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <sensor name="ray" type="altimeter">
          <always_on>1</always_on>
          <topic>/scan</topic>
          <update_rate>50.0</update_rate>
          <visualize>0</visualize>
          <ray>
            <noise type="none">
              <stddev>0.0</stddev>
              <bias_mean>0</bias_mean>
              <bias_stddev>0</bias_stddev>
              <precision>0</precision>
              <mean>0.0</mean>
            </noise>
            <scan>
              <vertical>
                <resolution>1.0</resolution>
                <min_angle>0.0</min_angle>
                <samples>1</samples>
                <max_angle>0.0</max_angle>
              </vertical>
              <horizontal>
                <resolution>1.0</resolution>
                <min_angle>-1.5707963267948966</min_angle>
                <samples>640</samples>
                <max_angle>1.5707963267948966</max_angle>
              </horizontal>
            </scan>
            <range>
              <max>10.0</max>
              <resolution>0.001</resolution>
              <min>0.05</min>
            </range>
          </ray>
        </sensor>
        <inertial>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <mass>0.01</mass>
          <inertia>
            <izz>1.0000000000000003e-05</izz>
            <ixx>1.0000000000000003e-05</ixx>
            <ixy>0.0</ixy>
            <iyz>0.0</iyz>
            <iyy>1.0000000000000003e-05</iyy>
            <ixz>0.0</ixz>
          </inertia>
        </inertial>
        <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      </link>
      <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      <allow_auto_disable>0</allow_auto_disable>
      <self_collide>0</self_collide>
      <static>0</static>
    </model>
    





    True




```python
print('List of ROS topics:')
for topic in simulation.get_rostopic_list():
    print(' - ' + topic)
```

    List of ROS topics:
     - /clock
     - /gazebo/link_states
     - /gazebo/model_states
     - /gazebo/parameter_descriptions
     - /gazebo/parameter_updates
     - /gazebo/set_link_state
     - /gazebo/set_model_state
     - /gazebo_gui/parameter_descriptions
     - /gazebo_gui/parameter_updates
     - /imu
     - /rosout
     - /rosout_agg
     - /tf
     - /tf_static


### Standalone contact sensor


```python
contact_sensor = SimulationModel(name='contact_standalone')

contact_sensor.add_contact_sensor(
    add_visual=True, 
    add_collision=True, 
    add_ros_plugin=True,
    mass=0.01,
    radius=0.05,
    length=0.1,
    link_shape='cylindrical',
    link_name='contact_link')

print(contact_sensor.to_sdf())

# Spawn ray sensor standalone model
generator.spawn_model(
    model=contact_sensor, 
    robot_namespace='contact_standalone',
    pos=[0, 0.6, 0.05])
```

    <model name="contact_standalone">
      <link name="contact_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.05</radius>
            </cylinder>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.05</radius>
            </cylinder>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <transparency>0.0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <sensor name="contact" type="contact">
          <always_on>1</always_on>
          <contact>
            <collision>collision</collision>
            <topic>/bumper</topic>
          </contact>
          <visualize>0</visualize>
          <topic>/bumper</topic>
          <update_rate>50.0</update_rate>
          <plugin filename="libgazebo_ros_bumper.so" name="contact">
            <bumperTopicName>/bumper</bumperTopicName>
            <frameName>world</frameName>
            <robotNamespace></robotNamespace>
          </plugin>
        </sensor>
        <inertial>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <mass>0.01</mass>
          <inertia>
            <izz>1.2500000000000002e-05</izz>
            <ixx>1.4583333333333333e-05</ixx>
            <ixy>0.0</ixy>
            <iyz>0.0</iyz>
            <iyy>1.4583333333333333e-05</iyy>
            <ixz>0.0</ixz>
          </inertia>
        </inertial>
        <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      </link>
      <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      <allow_auto_disable>0</allow_auto_disable>
      <self_collide>0</self_collide>
      <static>0</static>
    </model>
    





    True



### Standalone camera



```python
camera_sensor = SimulationModel(name='camera_standalone')
camera_sensor.static = True

camera_sensor.add_camera_sensor(
    add_visual=True, 
    add_collision=True, 
    add_ros_plugin=True,
    visualize=True,
    mass=0.01,
    size=[0.1, 0.1, 0.1],
    link_shape='cuboid',
    link_name='camera_link')

print(camera_sensor.to_sdf())

# Spawn camera standalone model
generator.spawn_model(
    model=camera_sensor, 
    robot_namespace='camera_standalone',
    pos=[0, 0.9, 0.5])
```

    <model name="camera_standalone">
      <link name="camera_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <transparency>0.0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <sensor name="camera" type="camera">
          <camera name="default">
            <save enabled="False">
              <path>__default__</path>
            </save>
            <clip>
              <far>100.0</far>
              <near>0.1</near>
            </clip>
            <noise type="none">
              <stddev>0.0</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.0</bias_stddev>
              <precision>0.0</precision>
              <mean>0.0</mean>
            </noise>
            <depth_camera>
              <output>depths</output>
            </depth_camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <height>240.0</height>
              <width>320.0</width>
              <format>R8G8B8</format>
            </image>
            <distortion>
              <k1>0.0</k1>
              <p1>0.0</p1>
              <k3>0.0</k3>
              <center>0.5 0.5</center>
              <k2>0.0</k2>
              <p2>0.0</p2>
            </distortion>
          </camera>
          <always_on>1</always_on>
          <visualize>1</visualize>
          <topic>/camera</topic>
          <update_rate>50.0</update_rate>
          <plugin filename="libgazebo_ros_camera.so" name="camera">
            <frameName>camera_link</frameName>
            <robotNamespace></robotNamespace>
            <imageTopicName>image_raw</imageTopicName>
            <distortionK3>0</distortionK3>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0</distortionK1>
            <distortionK2>0</distortionK2>
            <distortionT2>0</distortionT2>
            <updateRate>0</updateRate>
            <cameraName>camera</cameraName>
            <distortionT1>0</distortionT1>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          </plugin>
        </sensor>
        <inertial>
          <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
          <mass>0.01</mass>
          <inertia>
            <izz>1.6666666666666667e-05</izz>
            <ixx>1.6666666666666667e-05</ixx>
            <ixy>0.0</ixy>
            <iyz>0.0</iyz>
            <iyy>1.6666666666666667e-05</iyy>
            <ixz>0.0</ixz>
          </inertia>
        </inertial>
        <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      </link>
      <pose frame="">0 0 0 0.0 -0.0 0.0</pose>
      <allow_auto_disable>0</allow_auto_disable>
      <self_collide>0</self_collide>
      <static>1</static>
    </model>
    





    True




```python
print('List of ROS topics:')
for topic in simulation.get_rostopic_list():
    print(' - ' + topic)
```

    List of ROS topics:
     - /bumper
     - /clock
     - /gazebo/link_states
     - /gazebo/model_states
     - /gazebo/parameter_descriptions
     - /gazebo/parameter_updates
     - /gazebo/set_link_state
     - /gazebo/set_model_state
     - /gazebo_gui/parameter_descriptions
     - /gazebo_gui/parameter_updates
     - /imu
     - /rosout
     - /rosout_agg
     - /tf
     - /tf_static



```python
# End the simulation by killing the Gazebo task
simulation.kill_all_tasks()
```

![sensors](images/sensors.png)


```python

```

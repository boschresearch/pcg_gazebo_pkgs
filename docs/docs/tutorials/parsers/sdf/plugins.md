
# Plugins


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
plugin_args = dict(
    robotNamespace='robot',
    topic='/topic',
    args=dict(
        param1=1,
        param2=True,
        param3=dict(
            param31='option',
            param32=3.434534
        ),
        param4=[3.1, 4.5, 8.5]
    )
)
plugin = create_sdf_element('plugin', plugin_args)
plugin.name = 'some_plugin'
plugin.filename = 'libplugin.so'
print(plugin)
```

    <plugin filename="libplugin.so" name="some_plugin"/>
    



```python
plugin = create_sdf_element('plugin', plugin_args)
plugin.value = plugin_args
plugin.name = 'another_plugin'
plugin.filename = 'libanother_plugin.so'
print(plugin)
```

    <plugin filename="libanother_plugin.so" name="another_plugin">
      <topic>/topic</topic>
      <robotNamespace>robot</robotNamespace>
      <args>
        <param4>3.1 4.5 8.5</param4>
        <param1>1</param1>
        <param2>1</param2>
        <param3>
          <param31>option</param31>
          <param32>3.434534</param32>
        </param3>
      </args>
    </plugin>
    


# Some plugin default constructors


```python
from pcg_gazebo.parsers.sdf import Plugin
```

## `gazebo_ros_control`


```python
print(Plugin.gazebo_ros_control(
    name='gazebo_ros_control', 
    robot_namespace='/my_robot',
    control_period=10,
    robot_param='/robot_description',
    robot_sim_type=None))
```

    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
      <controlPeriod>10</controlPeriod>
      <robotNamespace>/my_robot</robotNamespace>
      <robotParam>/robot_description</robotParam>
    </plugin>
    


## `gazebo_ros_bumper`


```python
print(Plugin.gazebo_ros_bumper(
    name='gazebo_ros_bumper', 
    robot_namespace='/my_robot',
    bumper_topic_name='bumper_states',
    frame_name='world'))
```

    <plugin filename="libgazebo_ros_bumper.so" name="gazebo_ros_bumper">
      <bumperTopicName>bumper_states</bumperTopicName>
      <frameName>world</frameName>
      <robotNamespace>/my_robot</robotNamespace>
    </plugin>
    


## `gazebo_ros_ft_sensor`


```python
print(Plugin.gazebo_ros_ft_sensor(
    name='gazebo_ros_ft_sensor', 
    robot_namespace='my_robot',
    joint_name='some_joint', 
    topic_name='force_torque_sensor_output',
    gaussian_noise=0.05, 
    update_rate=0))
```

    <plugin filename="libgazebo_ros_ft_sensor.so" name="gazebo_ros_ft_sensor">
      <gaussianNoise>0.05</gaussianNoise>
      <jointName>some_joint</jointName>
      <updateRate>0</updateRate>
      <robotNamespace>my_robot</robotNamespace>
      <topicName>force_torque_sensor_output</topicName>
    </plugin>
    



# Process manager

The process manager allows the construction of sets of tasks to be run, which can be (but are not limited to) launch files or ROS nodes to bring up a system that may include an instance of Gazebo.
Each instance of the process manager can be set with a different `ROS_MASTER_URI` and `GAZEBO_MASTER_URI`, so that multiple `roscore` and `gazebo` instances can be initialized on the same host at the same time.

## Starting a process manager with default parameters


```python
from pcg_gazebo.task_manager import ProcessManager
# A process manager can be started by itself with an empty list of tasks
process_manager = ProcessManager()
# At this point neither Gazebo or roscore are running
# When no arguments are given, the default hostname and port
# arguments are going to be used.
print('ROS network configuration:')
print(process_manager.ros_config)
```

    ROS network configuration:
    ROS_MASTER_URI=http://localhost:11311, GAZEBO_MASTER_URI=http://localhost:11345



```python
# At first, no tasks are available in the tasks list
print(process_manager.get_task_list())
```

    []



```python
print('Check all the process stages available')
print(process_manager.stages)
```

    Check all the process stages available
    OrderedDict([('roscore', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21cc6855c0>), ('pre-simulation', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21876db2e8>)])



```python
# You can start roscore by calling the method below
process_manager.create_ros_core_task()
```




    True




```python
# Now the roscore task can be found in the list
print(process_manager.get_task_list())
```

    ['roscore']



```python
# The task can be started using the method below
# Running individual tasks will not use the stage order 
process_manager.run_task('roscore')
```


```python
# Check if the task is running
print('Is task running? {}'.format(process_manager.is_task_running('roscore')))
```

    Is task running? True



```python
# Creating an RViz task with a timeout
# The required flag, like the flag for ROS nodes, says that once this task dies, 
# all other tasks must be killed.
# The process timeout starts a timer and will kill the task in the amount of seconds
# given by process_timeout
# IMPORTANT: process_timeout is based on the machine clock, not the simulation clock
# process_timeout=None means that the process will run without a timeout
process_manager.create_rviz_task(required=True, process_timeout=10)
print(process_manager.get_task_list())
```

    ['rviz', 'roscore']



```python
# After 10 seconds all tasks will be killed along with rviz
process_manager.run_task('rviz')
process_manager.wait()

```


```python
process_manager.kill_all_tasks()
del process_manager
```

## Running Gazebo

Similar to RViz, Gazebo can also be started. The same process will be repeated to start Gazebo with the empty world scenario.


```python
# At first, no tasks are available in the tasks list
# A process manager can be started by itself with an empty list of tasks
# When ros_port and/or gazebo_port is given as None, a random port will be chosen
process_manager = ProcessManager(ros_port=None, gazebo_port=None)
print('ROS network configuration:')
print(process_manager.ros_config)
```

    ROS network configuration:
    ROS_MASTER_URI=http://localhost:17587, GAZEBO_MASTER_URI=http://localhost:28153



```python
# A Gazebo task can also be started with a process timeout
process_manager.create_gazebo_empty_world_task(required=True, process_timeout=10)
print('Check all tasks available')
print(process_manager.get_task_list())
print('Check all the process stages available')
print(process_manager.stages)
```

    Check all tasks available
    ['gazebo']
    Check all the process stages available
    OrderedDict([('roscore', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d31d0>), ('pre-simulation', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d3278>), ('gazebo', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d3dd8>)])



```python
process_manager.run_all_tasks()
# A roscore stage should be automatically added to the process manager list
print(process_manager.stages)
process_manager.wait()

```

    OrderedDict([('roscore', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d31d0>), ('pre-simulation', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d3278>), ('gazebo', <pcg_gazebo.task_manager.stage.Stage object at 0x7f21875d3dd8>)])



```python
process_manager.kill_all_tasks()
del process_manager

```


```python
# But a task that contains a Gazebo instance can also be started with a simulation timeout
# meaning that the process will be killed only when the simulation time reaches a timeout
# At first, no tasks are available in the tasks list
# A process manager can be started by itself with an empty list of tasks
# When ros_port and/or gazebo_port is given as None, a random port will be chosen
process_manager = ProcessManager(ros_port=None, gazebo_port=None)
print('ROS network configuration:')
print(process_manager.ros_config)

process_manager.create_gazebo_empty_world_task(required=True, simulation_timeout=10)
process_manager.run_all_tasks()
process_manager.wait()
```

    ROS network configuration:
    ROS_MASTER_URI=http://localhost:16226, GAZEBO_MASTER_URI=http://localhost:25435



```python
print('Is task running? {}'.format(process_manager.is_task_running('gazebo')))
```

    Is task running? False


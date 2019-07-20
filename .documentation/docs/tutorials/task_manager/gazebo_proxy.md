
# Gazebo proxy

The Gazebo proxy is an implementation of interfaces with all services provided by the `gazebo_ros_pkgs`. It allows easy use and from of the simulation through Python. 

It can be configured for different `ROS_MASTER_URI` and `GAZEBO_MASTER_URI` environment variables to access instances of Gazebo running in other hosts/ports.

The tutorial below will make use of the simulation manager to start instances of Gazebo.


```python
# Importing the Gazebo proxy
from pcg_gazebo.task_manager import GazeboProxy
```

The Gazebo proxy may also work with an instance of Gazebo that has been started external to the scope of this package, for example by running

```
roslaunch gazebo_ros empty_world.launch
```

The only instance will be found by using the input hostname and ports for which they are running. 
Here we will use the simulation manager.


```python
# If there is a Gazebo instance running, you can spawn the box into the simulation
from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('default')
simulation = server.get_simulation('default')
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with all default parameters
if not simulation.create_gazebo_empty_world_task():
    raise RuntimeError('Task for gazebo empty world could not be created')
```


```python
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
```


```python
# Run Gazebo
simulation.run_all_tasks()
```

Adding some models to the simulation to demonstrate the Gazebo proxy methods. 



```python
# Now create the Gazebo proxy with the default parameters. 
# If these input arguments are not provided, they will be used per default.
gazebo_proxy = simulation.get_gazebo_proxy()
# The timeout argument will be used raise an exception in case Gazebo 
# fails to start
```


```python
from pcg_gazebo.simulation import create_object
from pcg_gazebo.generators import WorldGenerator

generator = WorldGenerator(gazebo_proxy)

box = create_object('box')
box.add_inertial(mass=20)
print(box.to_sdf('model'))
```


```python
generator.spawn_model(
    model=box, 
    robot_namespace='box_1',
    pos=[-2, -2, 3])

generator.spawn_model(
    model=box, 
    robot_namespace='box_2',
    pos=[2, 2, 3])
```

## Pausing/unpausing the simulation



```python
from time import time, sleep
pause_timeout = 10 # seconds
start_time = time()
# Pausing simulation
gazebo_proxy.pause()
print('Simulation time before pause={}'.format(gazebo_proxy.sim_time))
while time() - start_time < pause_timeout:
    print('Gazebo paused, simulation time={}'.format(gazebo_proxy.sim_time))
    sleep(1)
print('Unpausing simulation!')
gazebo_proxy.unpause()
sleep(2)
print('Simulation time after pause={}'.format(gazebo_proxy.sim_time))
```

## Get world properties

The world properties return 

* Simulation time (`sim_time`)
* List of names of models (`model_names`)
* Is rendering enabled flag (`rendering_enabled`)

The return of this function is simply the service object [`GetWorldProperties`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/srv/GetWorldProperties.srv).


```python
# The world properties returns the following
gazebo_proxy.get_world_properties()
```

## Model properties


```python
# Get list of models
gazebo_proxy.get_model_names()
```


```python
# Get model properties
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_model_properties(model))
    print('-----------------')
```


```python
# Get model state
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_model_state(model_name=model, reference_frame='world'))
    print('-----------------')
```


```python
# Check if model exists
print('Does ground_plane exist? {}'.format(gazebo_proxy.model_exists('ground_plane')))
print('Does my_model exist? {}'.format(gazebo_proxy.model_exists('my_model')))
```


```python
# Get list of link names for a model
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_link_names(model))
    print('-----------------')
```


```python
# Test if model has a link
print('Does ground_plane have a link named link? {}'.format(gazebo_proxy.has_link(model_name='ground_plane', link_name='link')))
```


```python
# Get link properties
for model in gazebo_proxy.get_model_names():
    print(model)
    for link in gazebo_proxy.get_link_names(model_name=model):
        print(' - ' + link)
        print(gazebo_proxy.get_link_properties(model_name=model, link_name=link))
        print('-----------------')
    print('==================')
```


```python
# Get link state
for model in gazebo_proxy.get_model_names():
    print(model)
    for link in gazebo_proxy.get_link_names(model_name=model):
        print(' - ' + link)
        print(gazebo_proxy.get_link_state(model_name=model, link_name=link))
        print('-----------------')
    print('==================')
```

## Get physics properties

The physics properties returns the [GetPhysicsProperties](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/srv/GetPhysicsProperties.srv) response with the current parameters for the physics engine. Currently only the parameters for the ODE engine can be retrieved.


```python
print(gazebo_proxy.get_physics_properties())
```

## Apply wrench


```python
# Applying wrench to a link in the simulation
# The input arguments are
#  - model_name
#  - link_name
#  - force: force vector [x, y, z]
#  - torque: torque vector [x, y, z]
#  - start_time: in seconds, if it is a value lower than simulation time, the wrench will be applied as soon as possible
#  - duration: in seconds
#              if duration < 0, apply wrench continuously without end
#              if duration = 0, do nothing
#              if duration < step size, apply wrench for one step size
#  - reference_point: [x, y, z] coordinate point where wrench will be applied wrt the reference frame
#  - reference_frame: reference frame for the reference point, if None it will be set as the provided model_name::link_name
gazebo_proxy.apply_body_wrench(
    model_name='box_1',
    link_name='box',
    force=[100, 0, 0],
    torque=[0, 0, 100],
    start_time=0,
    duration=5,
    reference_point=[0, 0, 0],
    reference_frame=None)

gazebo_proxy.apply_body_wrench(
    model_name='box_2',
    link_name='box',
    force=[10, 0, 200],
    torque=[0, 0, 150],
    start_time=0,
    duration=4,
    reference_point=[0, 0, 0],
    reference_frame=None)

start_time = time()
while time() - start_time < 10:
    sleep(1)
```

## Move models in the simulation


```python
gazebo_proxy.move_model(
    model_name='box_1', 
    pos=[2, 2, 15],
    rot=[0, 0, 0],
    reference_frame='world')

gazebo_proxy.move_model(
    model_name='box_2', 
    pos=[-2, -1, 4],
    rot=[0, 0, 0],
    reference_frame='world')
```


```python
# End the simulation by killing the Gazebo task
simulation.kill_all_tasks()
```

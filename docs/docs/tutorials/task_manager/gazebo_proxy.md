
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

    ['gazebo']
    Is Gazebo running: False



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

    <model name="box">
      <static>0</static>
      <pose frame="">0 0 0 0.0 0.0 0.0</pose>
      <link name="box">
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <pose frame="">0 0 0 0.0 0.0 0.0</pose>
          <transparency>0.0</transparency>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <max_contacts>10</max_contacts>
          <pose frame="">0 0 0 0.0 0.0 0.0</pose>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <inertia>
            <ixz>0.0</ixz>
            <ixy>0.0</ixy>
            <iyy>3.333333333333333</iyy>
            <izz>3.333333333333333</izz>
            <iyz>0.0</iyz>
            <ixx>3.333333333333333</ixx>
          </inertia>
          <pose frame="">0 0 0 0.0 0.0 0.0</pose>
          <mass>20.0</mass>
        </inertial>
      </link>
      <allow_auto_disable>0</allow_auto_disable>
    </model>
    



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




    True



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

    Simulation time before pause=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Gazebo paused, simulation time=3.765
    Unpausing simulation!
    Simulation time after pause=5.758


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




    sim_time: 5.791
    model_names: - ground_plane
    - box_1
    - box_2
    rendering_enabled: True
    success: True
    status_message: "GetWorldProperties: got properties"



## Model properties


```python
# Get list of models
gazebo_proxy.get_model_names()
```




    ['ground_plane', 'box_1', 'box_2']




```python
# Get model properties
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_model_properties(model))
    print('-----------------')
```

    ground_plane
    parent_model_name: ''
    canonical_body_name: ''
    body_names: - link
    geom_names: - collision
    joint_names: []
    child_model_names: []
    is_static: True
    success: True
    status_message: "GetModelProperties: got properties"
    -----------------
    box_1
    parent_model_name: ''
    canonical_body_name: ''
    body_names: - box
    geom_names: - collision
    joint_names: []
    child_model_names: []
    is_static: False
    success: True
    status_message: "GetModelProperties: got properties"
    -----------------
    box_2
    parent_model_name: ''
    canonical_body_name: ''
    body_names: - box
    geom_names: - collision
    joint_names: []
    child_model_names: []
    is_static: False
    success: True
    status_message: "GetModelProperties: got properties"
    -----------------



```python
# Get model state
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_model_state(model_name=model, reference_frame='world'))
    print('-----------------')
```

    ground_plane
    header: 
      seq: 1
      stamp: 
        secs: 5
        nsecs: 875000000
      frame_id: "world"
    pose: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    twist: 
      linear: 
        x: 0.0
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
    success: True
    status_message: "GetModelState: got properties"
    -----------------
    box_1
    header: 
      seq: 1
      stamp: 
        secs: 5
        nsecs: 880000000
      frame_id: "world"
    pose: 
      position: 
        x: -2.0000000000000546
        y: -2.000000000000206
        z: 0.4999999999020089
      orientation: 
        x: 1.8990912910005212e-13
        y: -5.073873154687655e-14
        z: 1.3197537026938585e-15
        w: 1.0
    twist: 
      linear: 
        x: -2.3807002289237895e-10
        y: -2.3808787634481354e-10
        z: 9.799262257054186e-10
      angular: 
        x: 4.761622836684263e-10
        y: -4.761370405162758e-10
        z: 1.383497458711561e-15
    success: True
    status_message: "GetModelState: got properties"
    -----------------
    box_2
    header: 
      seq: 1
      stamp: 
        secs: 5
        nsecs: 886000000
      frame_id: "world"
    pose: 
      position: 
        x: 2.000000000000184
        y: 2.000000000000033
        z: 0.49999999990200894
      orientation: 
        x: -4.815307062778441e-14
        y: 1.8732345070772266e-13
        z: 1.429646856914762e-15
        w: 1.0
    twist: 
      linear: 
        x: 2.38087798121742e-10
        y: 2.38069852750414e-10
        z: 9.800284425098838e-10
      angular: 
        x: -4.7613708450024e-10
        y: 4.761622522897717e-10
        z: -1.828505909684487e-15
    success: True
    status_message: "GetModelState: got properties"
    -----------------



```python
# Check if model exists
print('Does ground_plane exist? {}'.format(gazebo_proxy.model_exists('ground_plane')))
print('Does my_model exist? {}'.format(gazebo_proxy.model_exists('my_model')))
```

    Does ground_plane exist? True
    Does my_model exist? False



```python
# Get list of link names for a model
for model in gazebo_proxy.get_model_names():
    print(model)
    print(gazebo_proxy.get_link_names(model))
    print('-----------------')
```

    ground_plane
    ['link']
    -----------------
    box_1
    ['box']
    -----------------
    box_2
    ['box']
    -----------------



```python
# Test if model has a link
print('Does ground_plane have a link named link? {}'.format(gazebo_proxy.has_link(model_name='ground_plane', link_name='link')))
```

    Does ground_plane have a link named link? True



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

    ground_plane
     - link
    com: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    gravity_mode: False
    mass: 0.0
    ixx: 0.0
    ixy: 0.0
    ixz: 0.0
    iyy: 0.0
    iyz: 0.0
    izz: 0.0
    success: True
    status_message: "GetLinkProperties: got properties"
    -----------------
    ==================
    box_1
     - box
    com: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    gravity_mode: True
    mass: 20.0
    ixx: 3.33333
    ixy: 0.0
    ixz: 0.0
    iyy: 3.33333
    iyz: 0.0
    izz: 3.33333
    success: True
    status_message: "GetLinkProperties: got properties"
    -----------------
    ==================
    box_2
     - box
    com: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    gravity_mode: True
    mass: 20.0
    ixx: 3.33333
    ixy: 0.0
    ixz: 0.0
    iyy: 3.33333
    iyz: 0.0
    izz: 3.33333
    success: True
    status_message: "GetLinkProperties: got properties"
    -----------------
    ==================



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

    ground_plane
     - link
    link_state: 
      link_name: "ground_plane::link"
      pose: 
        position: 
          x: 0.0
          y: 0.0
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
      twist: 
        linear: 
          x: 0.0
          y: 0.0
          z: 0.0
        angular: 
          x: 0.0
          y: 0.0
          z: 0.0
      reference_frame: ''
    success: True
    status_message: "GetLinkState: got state"
    -----------------
    ==================
    box_1
     - box
    link_state: 
      link_name: "box_1::box"
      pose: 
        position: 
          x: -2.0000000000000546
          y: -2.000000000000206
          z: 0.4999999999020089
        orientation: 
          x: 1.8990913102191923e-13
          y: -5.073873157309985e-14
          z: 1.3061809076207984e-15
          w: 1.0
      twist: 
        linear: 
          x: -2.380700244155512e-10
          y: -2.380878787093807e-10
          z: 9.799262257054186e-10
        angular: 
          x: 4.761622883975806e-10
          y: -4.761370435625982e-10
          z: 1.3834974541670622e-15
      reference_frame: ''
    success: True
    status_message: "GetLinkState: got state"
    -----------------
    ==================
    box_2
     - box
    link_state: 
      link_name: "box_2::box"
      pose: 
        position: 
          x: 2.000000000000184
          y: 2.000000000000033
          z: 0.49999999990200894
        orientation: 
          x: -4.815307081290838e-14
          y: 1.8732345239340608e-13
          z: 1.4160740618568299e-15
          w: 1.0
      twist: 
        linear: 
          x: 2.3808779897288484e-10
          y: 2.3806985173671113e-10
          z: 9.800284459793307e-10
        angular: 
          x: -4.761370824728559e-10
          y: 4.76162253992058e-10
          z: -1.8285059104865877e-15
      reference_frame: ''
    success: True
    status_message: "GetLinkState: got state"
    -----------------
    ==================


## Get physics properties

The physics properties returns the [GetPhysicsProperties](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/srv/GetPhysicsProperties.srv) response with the current parameters for the physics engine. Currently only the parameters for the ODE engine can be retrieved.


```python
print(gazebo_proxy.get_physics_properties())
```

    time_step: 0.001
    pause: False
    max_update_rate: 1000.0
    gravity: 
      x: 0.0
      y: 0.0
      z: -9.8
    ode_config: 
      auto_disable_bodies: False
      sor_pgs_precon_iters: 0
      sor_pgs_iters: 50
      sor_pgs_w: 1.3
      sor_pgs_rms_error_tol: 0.0
      contact_surface_layer: 0.001
      contact_max_correcting_vel: 100.0
      cfm: 0.0
      erp: 0.2
      max_contacts: 20
    success: True
    status_message: "GetPhysicsProperties: got properties"


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




    True




```python
# End the simulation by killing the Gazebo task
simulation.kill_all_tasks()
```

    2019-06-24 16:24:42,545 | ERROR | task | Child process 10636 still running
    2019-06-24 16:24:42,547 | ERROR | task | Child process 10672 still running
    2019-06-24 16:24:42,550 | ERROR | task | Child process 10677 still running
    2019-06-24 16:24:42,552 | ERROR | task | Child process 10750 still running
    2019-06-24 16:24:42,554 | ERROR | task | Child process 10738 still running
    2019-06-24 16:24:42,555 | ERROR | task | Child process 10635 still running
    2019-06-24 16:24:43,519 | ERROR | task | Child process 10420 still running
    2019-06-24 16:24:43,520 | ERROR | task | Child process 10531 still running
    2019-06-24 16:24:43,522 | ERROR | task | Child process 10548 still running
    2019-06-24 16:24:43,523 | ERROR | task | Child process 10419 still running


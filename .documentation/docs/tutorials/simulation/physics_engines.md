
# Physics engines

Gazebo has interfaces with four physics engines

* ODE
* Simbody
* Bullet
* DART

from which only DART has to be compiled separately.
The modules presented below allow the generation of the necessary physics engine's SDF parameters using classes.

## Physics engine object

The physics engine object holds the global parameters valid for all physics engines. 

If this modules is used to generate the SDF data, the default parameters of the physics engine chosen (`ode`, `bullet`, `simbody` or `dart`) will be used.


```python
from pcg_gazebo.simulation.physics import Physics
physics = Physics()

# Iterate through all parameters
for name in physics.get_parameter_names():
    print('{}: {}'.format(name, physics.get_parameter(name)))
    
```


```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```


```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))

```


```python
print(physics.to_sdf('world'))
```


```python
# Let's make a custom parameter set for the physics engine
physics.max_step_size = 0.01
physics.real_time_factor = 1
physics.real_time_update_rate = 500
physics.max_contacts = 5
physics.name = 'custom_physics'

print(physics.to_sdf())
```


```python
# The resulting world file can be exported to an .world file and run in Gazebo
# This shows how to create an SDF file from stratch
sdf = physics.to_sdf('sdf')
print(sdf)
```


```python
# Export to a .world file
world_filename = '/tmp/physics.world'
sdf.export_xml(filename=world_filename)
```


```python
# Start a PCG server to run the world file in Gazebo
from pcg_gazebo.task_manager import Server
server = Server()
# Create a simulation manager named default
server.create_simulation('default')
simulation = server.get_simulation('default')
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with the parameters provided to run the world file created
simulation.create_gazebo_task(
    name='gazebo',
    world=world_filename,
    gui=True,
    physics='ode',
    paused=False,
    required=True,
    process_timeout=10)
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()
```


```python
# Check if the parameters were initialized correctly
# We need a Gazebo proxy object to check it
from pcg_gazebo.task_manager import GazeboProxy
# Create a Gazebo proxy
gazebo_proxy = simulation.get_gazebo_proxy()

# It is important to note that the default get_physics_properties service from Gazebo
# returns only the global and the ODE engine parameters
print(gazebo_proxy.get_physics_properties())
```


```python
simulation.wait()
simulation.kill_all_tasks()
```

## ODE

It is possible to create an instance of the physics engine configuration for each engine available.

The ODE is presented on the following sections.


```python
from pcg_gazebo.simulation.physics import ODE
physics = ODE()

# Iterate through all parameters
for name in physics.get_parameter_names():
    print('{}: {}'.format(name, physics.get_parameter(name)))
```


```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```


```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```


```python
print(physics.to_sdf('world'))
```


```python
# Let's change some parameters
physics.max_step_size = 0.005
physics.friction_model = 'box_model'
physics.sor = 1.5
physics.max_contacts = 10
physics.name = 'custom_ode'
print(physics.to_sdf())
```


```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```


```python
# Export to a .world file
world_filename = '/tmp/physics_ode.world'
sdf.export_xml(filename=world_filename)
```


```python
# Remove old gazebo task
server.create_simulation('ode')
simulation = server.get_simulation('ode')
# Create new task
simulation.create_gazebo_task(
    name='gazebo',
    world=world_filename,
    gui=True,
    physics=physics.engine,
    paused=False,
    required=True,
    process_timeout=10)
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()
```


```python
# Check if the parameters were initialized correctly
gazebo_proxy = simulation.get_gazebo_proxy()

# It is important to note that the default get_physics_properties service from Gazebo
# returns only the global and the ODE engine parameters
print(gazebo_proxy.get_physics_properties())
```


```python
simulation.wait()
simulation.kill_all_tasks()
```

## Bullet


```python
from pcg_gazebo.simulation.physics import Bullet
physics = Bullet()

# Iterate through all parameters
for name in physics.get_parameter_names():
    print('{}: {}'.format(name, physics.get_parameter(name)))
```


```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```


```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```


```python
print(physics.to_sdf('world'))
```


```python
# Let's change some parameters
physics.max_step_size = 0.005
physics.cfm = 0.01
physics.sor = 1.5
physics.max_contacts = 5
physics.name = 'custom_bullet'
physics.real_time_update_rate = 500
print(physics.to_sdf())
```


```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```


```python
# Export to a .world file
world_filename = '/tmp/physics_bullet.world'
sdf.export_xml(filename=world_filename)
```


```python
# Remove old gazebo task
server.create_simulation('bullet')
simulation = server.get_simulation('bullet')
# Create new task
simulation.create_gazebo_task(
    name='gazebo',
    world=world_filename,
    gui=True,
    physics=physics.engine,
    paused=False,
    required=True,
    process_timeout=10)
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()

# The get_physics_properties service does not support bullet parameters yet
```


```python
simulation.wait()
simulation.kill_all_tasks()
```

## Simbody


```python
from pcg_gazebo.simulation.physics import Simbody
physics = Simbody()

# Iterate through all parameters
for name in physics.get_parameter_names():
    print('{}: {}'.format(name, physics.get_parameter(name)))
```


```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```


```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```


```python
print(physics.to_sdf('world'))
```


```python
# Let's change some parameters
physics.max_step_size = 0.005
physics.max_contacts = 8
physics.name = 'custom_simbody'
physics.static_friction = 1.0
physics.real_time_update_rate = 500
print(physics.to_sdf())
```


```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```


```python
# Export to a .world file
world_filename = '/tmp/physics_simbody.world'
sdf.export_xml(filename=world_filename)
```


```python
server.create_simulation('simbody')
simulation = server.get_simulation('simbody')
# Create new task
simulation.create_gazebo_task(
    name='gazebo',
    world=world_filename,
    gui=True,
    physics=physics.engine,
    paused=False,
    required=True,
    process_timeout=10)
# A task named 'gazebo' the added to the tasks list
print(simulation.get_task_list())
# But it is still not running
print('Is Gazebo running: {}'.format(simulation.is_task_running('gazebo')))
# Run Gazebo
simulation.run_all_tasks()

# The get_physics_properties service does not support simbody parameters yet
```


```python
simulation.wait()
simulation.kill_all_tasks()
```

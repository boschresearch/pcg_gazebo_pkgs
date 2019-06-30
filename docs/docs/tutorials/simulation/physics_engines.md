
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

    name: default_physics
    real_time_update_rate: 1000
    real_time_factor: 1
    engine: ode
    default: False
    max_step_size: 0.001
    max_contacts: 20



```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```

    name
      Description: The name of this set of physics parameters
      Current value: default_physics
    real_time_update_rate
      Description: Rate at which to update the physics engine (UpdatePhysics calls per real-time second)
      Current value: 1000
    real_time_factor
      Description: Target simulation speedupfactor, defined by ratio of simulation time to real-time
      Current value: 1
    engine
      Description: The type of the dynamics engine. Current options are ode, bullet, simbody and dart. Defaults to ode if left unspecified.
      Current value: ode
    default
      Description: If true, this physics element is set as the default physics profile for the world. If multiple default physics elements exist, the first element marked as default is chosen. If no default physics element exists, the first physics element is chosen.
      Current value: False
    max_step_size
      Description: Maximum time step size at which every system in simulation can interact with the states of the world
      Current value: 0.001
    max_contacts
      Description: Maximum number of contactsallowed between two entities. This value can be over ridden by a max_contacts element in a collision element
      Current value: 20



```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))

```

    <physics default="1" name="default_physics" type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>20</max_contacts>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    



```python
print(physics.to_sdf('world'))
```

    <world name="default">
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1.0</real_time_factor>
        <max_contacts>20</max_contacts>
        <real_time_update_rate>1000.0</real_time_update_rate>
        <max_step_size>0.001</max_step_size>
      </physics>
      <gravity>0 0 -9.8</gravity>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
    </world>
    



```python
# Let's make a custom parameter set for the physics engine
physics.max_step_size = 0.01
physics.real_time_factor = 1
physics.real_time_update_rate = 500
physics.max_contacts = 5
physics.name = 'custom_physics'

print(physics.to_sdf())
```

    <physics default="1" name="custom_physics" type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>5</max_contacts>
      <real_time_update_rate>500.0</real_time_update_rate>
      <max_step_size>0.01</max_step_size>
    </physics>
    



```python
# The resulting world file can be exported to an .world file and run in Gazebo
# This shows how to create an SDF file from stratch
sdf = physics.to_sdf('sdf')
print(sdf)
```

    <sdf version="1.6">
      <world name="default">
        <physics default="1" name="custom_physics" type="ode">
          <real_time_factor>1.0</real_time_factor>
          <max_contacts>5</max_contacts>
          <real_time_update_rate>500.0</real_time_update_rate>
          <max_step_size>0.01</max_step_size>
        </physics>
        <gravity>0 0 -9.8</gravity>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
      </world>
    </sdf>
    



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

    ['gazebo']
    Is Gazebo running: False



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

    time_step: 0.01
    pause: False
    max_update_rate: 500.0
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
      max_contacts: 5
    success: True
    status_message: "GetPhysicsProperties: got properties"



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

    min_step_size: 0.0001
    precon_iters: 0
    name: default_physics
    real_time_update_rate: 1000
    friction_model: pyramid_model
    real_time_factor: 1
    engine: ode
    default: False
    max_step_size: 0.001
    type: quick
    sor: 1.3
    use_dynamic_moi_scaling: False
    max_contacts: 20
    contact_max_correcting_vel: 100
    contact_surface_layer: 0.001
    erp: 0.2
    iters: 50
    cfm: 0



```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```

    min_step_size
      Description: The time duration which advances with each iteration of the dynamics engine, this has to be no bigger than max_step_size under physics block. If left unspecified, min_step_size defaults to max_step_size
      Current value: 0.0001
    precon_iters
      Description: Experimental parameter
      Current value: 0
    name
      Description: The name of this set of physics parameters
      Current value: default_physics
    real_time_update_rate
      Description: Rate at which to update the physics engine (UpdatePhysics calls per real-time second)
      Current value: 1000
    friction_model
      Description: Name of ODE friction model to use. Valid values include: pyramid_model: (default) friction forces limited in two directions in proportion to normal force. box_model: friction forces limited to constant in two directions. cone_model: friction force magnitude limited in proportion to normal force
      Current value: pyramid_model
    real_time_factor
      Description: Target simulation speedupfactor, defined by ratio of simulation time to real-time
      Current value: 1
    engine
      Description: The type of the dynamics engine. Current options are ode, bullet, simbody and dart. Defaults to ode if left unspecified.
      Current value: ode
    default
      Description: If true, this physics element is set as the default physics profile for the world. If multiple default physics elements exist, the first element marked as default is chosen. If no default physics element exists, the first physics element is chosen.
      Current value: False
    max_step_size
      Description: Maximum time step size at which every system in simulation can interact with the states of the world
      Current value: 0.001
    type
      Description: One of the following types: world, quick
      Current value: quick
    sor
      Description: Set the successive over-relaxation parameter.
      Current value: 1.3
    use_dynamic_moi_scaling
      Description: Flag to enable dynamic rescaling of moment of inertia in constrained directions
      Current value: False
    max_contacts
      Description: Maximum number of contactsallowed between two entities. This value can be over ridden by a max_contacts element in a collision element
      Current value: 20
    contact_max_correcting_vel
      Description: The maximum correcting velocities allowed when resolving contacts
      Current value: 100
    contact_surface_layer
      Description: The depth of the surface layer around all geometry objects. Contacts are allowed to sink into the surface layer up to the given depth before coming to rest. The default value is zero. Increasing this to some small value (e.g. 0.001) can help prevent jittering problems due to contacts being repeatedly made and broken.
      Current value: 0.001
    erp
      Description: Error reduction parameter
      Current value: 0.2
    iters
      Description: Number of iterations for each step. A higher number produces greater accuracy at a performance cost.
      Current value: 50
    cfm
      Description: Constraint force mixing parameter
      Current value: 0



```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```

    <physics default="1" name="default_physics" type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>20</max_contacts>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <ode>
        <constraints>
          <erp>0.2</erp>
          <contact_surface_layer>0.001</contact_surface_layer>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <cfm>0.0</cfm>
        </constraints>
        <solver>
          <min_step_size>0.0001</min_step_size>
          <friction_model>pyramid_model</friction_model>
          <sor>1.3</sor>
          <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
          <precon_iters>0</precon_iters>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>
    



```python
print(physics.to_sdf('world'))
```

    <world name="default">
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1.0</real_time_factor>
        <max_contacts>20</max_contacts>
        <real_time_update_rate>1000.0</real_time_update_rate>
        <max_step_size>0.001</max_step_size>
        <ode>
          <constraints>
            <erp>0.2</erp>
            <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
            <contact_surface_layer>0.001</contact_surface_layer>
            <cfm>0.0</cfm>
          </constraints>
          <solver>
            <min_step_size>0.0001</min_step_size>
            <friction_model>pyramid_model</friction_model>
            <sor>1.3</sor>
            <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
            <precon_iters>0</precon_iters>
            <type>quick</type>
            <iters>50</iters>
          </solver>
        </ode>
      </physics>
      <gravity>0 0 -9.8</gravity>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
    </world>
    



```python
# Let's change some parameters
physics.max_step_size = 0.005
physics.friction_model = 'box_model'
physics.sor = 1.5
physics.max_contacts = 10
physics.name = 'custom_ode'
print(physics.to_sdf())
```

    <physics default="1" name="custom_ode" type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>10</max_contacts>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.005</max_step_size>
      <ode>
        <constraints>
          <erp>0.2</erp>
          <contact_surface_layer>0.001</contact_surface_layer>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <cfm>0.0</cfm>
        </constraints>
        <solver>
          <min_step_size>0.0001</min_step_size>
          <friction_model>box_model</friction_model>
          <sor>1.5</sor>
          <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
          <precon_iters>0</precon_iters>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>
    



```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```

    <sdf version="1.6">
      <world name="default">
        <physics default="1" name="custom_ode" type="ode">
          <real_time_factor>1.0</real_time_factor>
          <max_contacts>10</max_contacts>
          <real_time_update_rate>1000.0</real_time_update_rate>
          <max_step_size>0.005</max_step_size>
          <ode>
            <constraints>
              <erp>0.2</erp>
              <contact_surface_layer>0.001</contact_surface_layer>
              <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
              <cfm>0.0</cfm>
            </constraints>
            <solver>
              <min_step_size>0.0001</min_step_size>
              <friction_model>box_model</friction_model>
              <sor>1.5</sor>
              <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
              <precon_iters>0</precon_iters>
              <type>quick</type>
              <iters>50</iters>
            </solver>
          </ode>
        </physics>
        <gravity>0 0 -9.8</gravity>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
      </world>
    </sdf>
    



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

    ['gazebo']
    Is Gazebo running: False



```python
# Check if the parameters were initialized correctly
gazebo_proxy = simulation.get_gazebo_proxy()

# It is important to note that the default get_physics_properties service from Gazebo
# returns only the global and the ODE engine parameters
print(gazebo_proxy.get_physics_properties())
```

    time_step: 0.005
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
      sor_pgs_w: 1.5
      sor_pgs_rms_error_tol: 0.0
      contact_surface_layer: 0.001
      contact_max_correcting_vel: 100.0
      cfm: 0.0
      erp: 0.2
      max_contacts: 10
    success: True
    status_message: "GetPhysicsProperties: got properties"



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

    min_step_size: 0.0001
    split_impulse_penetration_threshold: -0.01
    name: default_physics
    real_time_update_rate: 1000
    real_time_factor: 1
    engine: bullet
    default: False
    max_step_size: 0.001
    type: sequential_impulse
    sor: 1.3
    max_contacts: 20
    contact_surface_layer: 0.001
    erp: 0.2
    split_impulse: True
    iters: 50
    cfm: 0



```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```

    min_step_size
      Description: The time duration which advances with each iteration of the dynamics engine, this has to be no bigger than max_step_size under physics block. If left unspecified, min_step_size defaults to max_step_size
      Current value: 0.0001
    split_impulse_penetration_threshold
      Description: Similarto ODE max_vel implementation
      Current value: -0.01
    name
      Description: The name of this set of physics parameters
      Current value: default_physics
    real_time_update_rate
      Description: Rate at which to update the physics engine (UpdatePhysics calls per real-time second)
      Current value: 1000
    real_time_factor
      Description: Target simulation speedupfactor, defined by ratio of simulation time to real-time
      Current value: 1
    engine
      Description: The type of the dynamics engine. Current options are ode, bullet, simbody and dart. Defaults to ode if left unspecified.
      Current value: bullet
    default
      Description: If true, this physics element is set as the default physics profile for the world. If multiple default physics elements exist, the first element marked as default is chosen. If no default physics element exists, the first physics element is chosen.
      Current value: False
    max_step_size
      Description: Maximum time step size at which every system in simulation can interact with the states of the world
      Current value: 0.001
    type
      Description: One of the following types: sequential_impulse only
      Current value: sequential_impulse
    sor
      Description: Set the successive over-relaxation parameter.
      Current value: 1.3
    max_contacts
      Description: Maximum number of contactsallowed between two entities. This value can be over ridden by a max_contacts element in a collision element
      Current value: 20
    contact_surface_layer
      Description: The depth of the surface layer around all geometry objects. Contacts are allowed to sink into the surface layer up to the given depth before coming to rest. The default value is zero. Increasing this to some small value (e.g. 0.001) can help prevent jittering problems due to contacts being repeatedly made and broken.
      Current value: 0.001
    erp
      Description: Error reduction parameter
      Current value: 0.2
    split_impulse
      Description: Similar to ODE max_vel implementation
      Current value: True
    iters
      Description: Number of iterations for each step. A higher number produces greater accuracy at a performance cost.
      Current value: 50
    cfm
      Description: Constraint force mixing parameter
      Current value: 0



```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```

    <physics default="1" name="default_physics" type="bullet">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>20</max_contacts>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <bullet>
        <constraints>
          <erp>0.2</erp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <split_impulse>1</split_impulse>
          <contact_surface_layer>0.001</contact_surface_layer>
          <cfm>0.0</cfm>
        </constraints>
        <solver>
          <min_step_size>0.0001</min_step_size>
          <type>quick</type>
          <sor>1.3</sor>
          <iters>50</iters>
        </solver>
      </bullet>
    </physics>
    



```python
print(physics.to_sdf('world'))
```

    <world name="default">
      <physics default="1" name="default_physics" type="bullet">
        <real_time_factor>1.0</real_time_factor>
        <max_contacts>20</max_contacts>
        <real_time_update_rate>1000.0</real_time_update_rate>
        <max_step_size>0.001</max_step_size>
        <bullet>
          <constraints>
            <erp>0.2</erp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <split_impulse>1</split_impulse>
            <contact_surface_layer>0.001</contact_surface_layer>
            <cfm>0.0</cfm>
          </constraints>
          <solver>
            <min_step_size>0.0001</min_step_size>
            <type>quick</type>
            <sor>1.3</sor>
            <iters>50</iters>
          </solver>
        </bullet>
      </physics>
      <gravity>0 0 -9.8</gravity>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
    </world>
    



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

    <physics default="1" name="custom_bullet" type="bullet">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>5</max_contacts>
      <real_time_update_rate>500.0</real_time_update_rate>
      <max_step_size>0.005</max_step_size>
      <bullet>
        <constraints>
          <erp>0.2</erp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <split_impulse>1</split_impulse>
          <contact_surface_layer>0.001</contact_surface_layer>
          <cfm>0.01</cfm>
        </constraints>
        <solver>
          <min_step_size>0.0001</min_step_size>
          <type>quick</type>
          <sor>1.5</sor>
          <iters>50</iters>
        </solver>
      </bullet>
    </physics>
    



```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```

    <sdf version="1.6">
      <world name="default">
        <physics default="1" name="custom_bullet" type="bullet">
          <real_time_factor>1.0</real_time_factor>
          <max_contacts>5</max_contacts>
          <real_time_update_rate>500.0</real_time_update_rate>
          <max_step_size>0.005</max_step_size>
          <bullet>
            <constraints>
              <erp>0.2</erp>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <split_impulse>1</split_impulse>
              <contact_surface_layer>0.001</contact_surface_layer>
              <cfm>0.01</cfm>
            </constraints>
            <solver>
              <min_step_size>0.0001</min_step_size>
              <type>quick</type>
              <sor>1.5</sor>
              <iters>50</iters>
            </solver>
          </bullet>
        </physics>
        <gravity>0 0 -9.8</gravity>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
      </world>
    </sdf>
    



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

    ['gazebo']
    Is Gazebo running: False



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

    min_step_size: 0.0001
    viscous_friction: 0.9
    dissipation: 100
    name: default_physics
    max_transient_velocity: 0.01
    accuracy: 0.001
    plastic_coef_restitution: 0.5
    real_time_factor: 1
    engine: simbody
    stiffness: 100000000.0
    max_step_size: 0.001
    plastic_impact_velocity: 0.5
    default: False
    max_contacts: 20
    override_stiction_transition_velocity: 0.001
    static_friction: 0.9
    override_impact_capture_velocity: 0.001
    dynamic_friction: 0.9
    real_time_update_rate: 1000



```python
# The description of the parameters is also available
for name in physics.get_parameter_names():
    physics.print_description(name)
```

    min_step_size
      Description: (Currently not used in simbody) The time duration which advances with each iteration of the dynamics engine, this has to be no bigger than max_step_size under physics block. If left unspecified, min_step_size defaults to max_step_size
      Current value: 0.0001
    viscous_friction
      Description: Viscous friction [mu_v] with units of 1 / velocity
      Current value: 0.9
    dissipation
      Description: Dissipation coefficient to be used in compliant contact; if not given it is (1-min_cor)/plastic_impact_velocity
      Current value: 100
    name
      Description: The name of this set of physics parameters
      Current value: default_physics
    max_transient_velocity
      Description: Tolerable "slip" velocity allowed by the solver when static friction is supposed to hold object in place.
      Current value: 0.01
    accuracy
      Description: Roughly the relative error of the system. -LOG(accuracy) is roughly the number of significant digits.
      Current value: 0.001
    plastic_coef_restitution
      Description: This is the COR to be used at high velocities for rigid impacts; if not given it is 1 - dissipation*plastic_impact_velocity
      Current value: 0.5
    real_time_factor
      Description: Target simulation speedupfactor, defined by ratio of simulation time to real-time
      Current value: 1
    engine
      Description: The type of the dynamics engine. Current options are ode, bullet, simbody and dart. Defaults to ode if left unspecified.
      Current value: simbody
    stiffness
      Description: Default contact material stiffness (force/dist or torque/radian).
      Current value: 100000000.0
    max_step_size
      Description: Maximum time step size at which every system in simulation can interact with the states of the world
      Current value: 0.001
    plastic_impact_velocity
      Description: Smallest impact velocity at which min COR is reached; set to zero if you want the min COR always to be used
      Current value: 0.5
    default
      Description: If true, this physics element is set as the default physics profile for the world. If multiple default physics elements exist, the first element marked as default is chosen. If no default physics element exists, the first physics element is chosen.
      Current value: False
    max_contacts
      Description: Maximum number of contactsallowed between two entities. This value can be over ridden by a max_contacts element in a collision element
      Current value: 20
    override_stiction_transition_velocity
      Description: This is the largest slip velocity at which we will consider a transition to stiction. Normally inherited from a global default setting. For a continuous friction model this is the velocity at which the max static friction force is reached. Combining rule: use larger velocity
      Current value: 0.001
    static_friction
      Description: Static friction [mu_s]
      Current value: 0.9
    override_impact_capture_velocity
      Description: For rigid impacts only, impact velocity at which COR is set to zero; normally inherited from global default but can be overridden here. Combining rule: use larger velocity
      Current value: 0.001
    dynamic_friction
      Description: Dynamic friction [mu_d]
      Current value: 0.9
    real_time_update_rate
      Description: Rate at which to update the physics engine (UpdatePhysics calls per real-time second)
      Current value: 1000



```python
# It is also possible to generate the SDF files.
# The SDF object can also be retrieved and altered if necessary
print(physics.to_sdf('physics'))
```

    <physics default="1" name="default_physics" type="simbody">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>20</max_contacts>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <simbody>
        <min_step_size>0.0001</min_step_size>
        <contact>
          <viscous_friction>0.9</viscous_friction>
          <plastic_coef_restitution>0.5</plastic_coef_restitution>
          <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
          <static_friction>0.9</static_friction>
          <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
          <dynamic_friction>0.9</dynamic_friction>
          <dissipation>100.0</dissipation>
          <stiffness>100000000.0</stiffness>
          <plastic_impact_velocity>0.5</plastic_impact_velocity>
        </contact>
        <accuracy>0.001</accuracy>
        <max_transient_velocity>0.01</max_transient_velocity>
      </simbody>
      <max_step_size>0.001</max_step_size>
    </physics>
    



```python
print(physics.to_sdf('world'))
```

    <world name="default">
      <physics default="1" name="default_physics" type="simbody">
        <real_time_factor>1.0</real_time_factor>
        <max_contacts>20</max_contacts>
        <real_time_update_rate>1000.0</real_time_update_rate>
        <simbody>
          <min_step_size>0.0001</min_step_size>
          <contact>
            <viscous_friction>0.9</viscous_friction>
            <plastic_impact_velocity>0.5</plastic_impact_velocity>
            <plastic_coef_restitution>0.5</plastic_coef_restitution>
            <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
            <static_friction>0.9</static_friction>
            <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
            <dissipation>100.0</dissipation>
            <stiffness>100000000.0</stiffness>
            <dynamic_friction>0.9</dynamic_friction>
          </contact>
          <accuracy>0.001</accuracy>
          <max_transient_velocity>0.01</max_transient_velocity>
        </simbody>
        <max_step_size>0.001</max_step_size>
      </physics>
      <gravity>0 0 -9.8</gravity>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://sun</uri>
      </include>
    </world>
    



```python
# Let's change some parameters
physics.max_step_size = 0.005
physics.max_contacts = 8
physics.name = 'custom_simbody'
physics.static_friction = 1.0
physics.real_time_update_rate = 500
print(physics.to_sdf())
```

    <physics default="1" name="custom_simbody" type="simbody">
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>8</max_contacts>
      <real_time_update_rate>500.0</real_time_update_rate>
      <simbody>
        <min_step_size>0.0001</min_step_size>
        <contact>
          <viscous_friction>0.9</viscous_friction>
          <plastic_coef_restitution>0.5</plastic_coef_restitution>
          <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
          <static_friction>1.0</static_friction>
          <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
          <dynamic_friction>0.9</dynamic_friction>
          <dissipation>100.0</dissipation>
          <stiffness>100000000.0</stiffness>
          <plastic_impact_velocity>0.5</plastic_impact_velocity>
        </contact>
        <accuracy>0.001</accuracy>
        <max_transient_velocity>0.01</max_transient_velocity>
      </simbody>
      <max_step_size>0.005</max_step_size>
    </physics>
    



```python
# Exporting this world configuration to a file allows running the 
# configured physics engine in Gazebo
sdf = physics.to_sdf('sdf')
print(sdf)
```

    <sdf version="1.6">
      <world name="default">
        <physics default="1" name="custom_simbody" type="simbody">
          <real_time_factor>1.0</real_time_factor>
          <max_contacts>8</max_contacts>
          <real_time_update_rate>500.0</real_time_update_rate>
          <simbody>
            <min_step_size>0.0001</min_step_size>
            <contact>
              <viscous_friction>0.9</viscous_friction>
              <plastic_coef_restitution>0.5</plastic_coef_restitution>
              <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
              <static_friction>1.0</static_friction>
              <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
              <dynamic_friction>0.9</dynamic_friction>
              <dissipation>100.0</dissipation>
              <stiffness>100000000.0</stiffness>
              <plastic_impact_velocity>0.5</plastic_impact_velocity>
            </contact>
            <accuracy>0.001</accuracy>
            <max_transient_velocity>0.01</max_transient_velocity>
          </simbody>
          <max_step_size>0.005</max_step_size>
        </physics>
        <gravity>0 0 -9.8</gravity>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
      </world>
    </sdf>
    



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

    ['gazebo']
    Is Gazebo running: False



```python
simulation.wait()
simulation.kill_all_tasks()
```

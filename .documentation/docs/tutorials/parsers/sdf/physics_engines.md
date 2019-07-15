
# Physics engines

The description of the physics engine's parameters is one of the most import parts in the world description in Gazebo. A `<world>` can only have one physics element.

It can use the following engines: 

* ODE (`ode`)
* Bullet (`bullet`)
* Simbody (`simbody`)
* DART (`dart`) 

and a specific SDF block is available to describe each engine.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# Create first the global physics block
physics = create_sdf_element('physics')
print(physics)
```

    <physics default="1" name="default_physics" type="ode">
      <max_contacts>20</max_contacts>
      <real_time_factor>1</real_time_factor>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    



```python
# The physics engine's configuration modes are named after the 
# engine being used, the default being `ode`
physics.reset(mode='ode', with_optional_elements=True)
print(physics)
```

    <physics default="1" name="default_physics" type="ode">
      <max_contacts>20</max_contacts>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
      <max_step_size>0.001</max_step_size>
      <ode>
        <solver>
          <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
          <sor>1.3</sor>
          <friction_model>pyramid_model</friction_model>
          <type>quick</type>
          <iters>50</iters>
          <min_step_size>0.0001</min_step_size>
          <precon_iters>0</precon_iters>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>
    



```python
physics.reset(mode='bullet', with_optional_elements=True)
print(physics)
```

    <physics default="1" name="default_physics" type="bullet">
      <max_contacts>20</max_contacts>
      <bullet>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
          <min_step_size>0.0001</min_step_size>
        </solver>
        <constraints>
          <split_impulse>1</split_impulse>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </bullet>
      <real_time_factor>1</real_time_factor>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    



```python
physics.reset(mode='simbody', with_optional_elements=True)
print(physics)
```

    <physics default="1" name="default_physics" type="simbody">
      <simbody>
        <accuracy>0.001</accuracy>
        <contact>
          <plastic_impact_velocity>0.5</plastic_impact_velocity>
          <viscous_friction>0</viscous_friction>
          <dissipation>100</dissipation>
          <stiffness>100000000.0</stiffness>
          <plastic_coef_restitution>0.5</plastic_coef_restitution>
          <static_friction>0.9</static_friction>
          <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
          <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
          <dynamic_friction>0.9</dynamic_friction>
        </contact>
        <max_transient_velocity>0.01</max_transient_velocity>
        <min_step_size>0.0001</min_step_size>
      </simbody>
      <max_contacts>20</max_contacts>
      <real_time_factor>1</real_time_factor>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    



```python

```

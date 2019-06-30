
# World

The world element encapsulates an entire world description including: models, scene, physics, joints, and plugins.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# The default world contains only the definition of the acceleration of gravity 
# and the default configuration of the physics engine using `ode`
world = create_sdf_element('world')
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
      </physics>
    </world>
    



```python
# The physics engine can be edited using its object setter
world.physics.reset(mode='ode', with_optional_elements=True)
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <max_contacts>20</max_contacts>
        <ode>
          <constraints>
            <contact_max_correcting_vel>100</contact_max_correcting_vel>
            <cfm>0</cfm>
            <contact_surface_layer>0.001</contact_surface_layer>
            <erp>0.2</erp>
          </constraints>
          <solver>
            <iters>50</iters>
            <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
            <sor>1.3</sor>
            <friction_model>pyramid_model</friction_model>
            <min_step_size>0.0001</min_step_size>
            <type>quick</type>
            <precon_iters>0</precon_iters>
          </solver>
        </ode>
        <max_step_size>0.001</max_step_size>
      </physics>
    </world>
    



```python
world.physics.reset(mode='bullet', with_optional_elements=True)
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="bullet">
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <bullet>
          <constraints>
            <cfm>0</cfm>
            <erp>0.2</erp>
            <contact_surface_layer>0.001</contact_surface_layer>
            <split_impulse>1</split_impulse>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </constraints>
          <solver>
            <min_step_size>0.0001</min_step_size>
            <iters>50</iters>
            <sor>1.3</sor>
            <type>quick</type>
          </solver>
        </bullet>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
      </physics>
    </world>
    



```python
world.physics.reset(mode='simbody', with_optional_elements=True)
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="simbody">
        <simbody>
          <max_transient_velocity>0.01</max_transient_velocity>
          <contact>
            <stiffness>100000000.0</stiffness>
            <static_friction>0.9</static_friction>
            <plastic_impact_velocity>0.5</plastic_impact_velocity>
            <dissipation>100</dissipation>
            <viscous_friction>0</viscous_friction>
            <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
            <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
            <dynamic_friction>0.9</dynamic_friction>
            <plastic_coef_restitution>0.5</plastic_coef_restitution>
          </contact>
          <accuracy>0.001</accuracy>
          <min_step_size>0.0001</min_step_size>
        </simbody>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
      </physics>
    </world>
    



```python
# A world can contain multiple models, add an empty model by using the add_model method
world.reset()
world.add_model('model_1')
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
      </physics>
      <model name="model_1"/>
    </world>
    



```python
# You can add a model to the world as well
model = create_sdf_element('model')
world.add_model('model_2', model)
print(world)
```

    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <physics default="1" name="default_physics" type="ode">
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
      </physics>
      <model name="model_1"/>
      <model name="model_2"/>
    </world>
    



```python
# You must have unique names for the models in a world
world.add_model('model_1')

```

    Model element with name model_1 already exists



```python

```


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
      <physics default="1" name="default_physics" type="ode">
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <gravity>0 0 -9.8</gravity>
    </world>
    



```python
# The physics engine can be edited using its object setter
world.physics.reset(mode='ode', with_optional_elements=True)
print(world)
```

    <world name="default">
      <physics default="1" name="default_physics" type="ode">
        <ode>
          <constraints>
            <cfm>0</cfm>
            <erp>0.2</erp>
            <contact_surface_layer>0.001</contact_surface_layer>
            <contact_max_correcting_vel>100</contact_max_correcting_vel>
          </constraints>
          <solver>
            <type>quick</type>
            <sor>1.3</sor>
            <use_dynamic_moi_scaling>0</use_dynamic_moi_scaling>
            <friction_model>pyramid_model</friction_model>
            <min_step_size>0.0001</min_step_size>
            <precon_iters>0</precon_iters>
            <iters>50</iters>
          </solver>
        </ode>
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <gravity>0 0 -9.8</gravity>
    </world>
    



```python
world.physics.reset(mode='bullet', with_optional_elements=True)
print(world)
```

    <world name="default">
      <physics default="1" name="default_physics" type="bullet">
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
        <bullet>
          <constraints>
            <cfm>0</cfm>
            <split_impulse>1</split_impulse>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <contact_surface_layer>0.001</contact_surface_layer>
            <erp>0.2</erp>
          </constraints>
          <solver>
            <type>quick</type>
            <sor>1.3</sor>
            <iters>50</iters>
            <min_step_size>0.0001</min_step_size>
          </solver>
        </bullet>
        <real_time_update_rate>1000</real_time_update_rate>
        <real_time_factor>1</real_time_factor>
      </physics>
      <gravity>0 0 -9.8</gravity>
    </world>
    



```python
world.physics.reset(mode='simbody', with_optional_elements=True)
print(world)
```

    <world name="default">
      <physics default="1" name="default_physics" type="simbody">
        <max_contacts>20</max_contacts>
        <simbody>
          <accuracy>0.001</accuracy>
          <min_step_size>0.0001</min_step_size>
          <contact>
            <plastic_impact_velocity>0.5</plastic_impact_velocity>
            <plastic_coef_restitution>0.5</plastic_coef_restitution>
            <stiffness>100000000.0</stiffness>
            <dissipation>100</dissipation>
            <override_stiction_transition_velocity>0.001</override_stiction_transition_velocity>
            <viscous_friction>0</viscous_friction>
            <dynamic_friction>0.9</dynamic_friction>
            <static_friction>0.9</static_friction>
            <override_impact_capture_velocity>0.001</override_impact_capture_velocity>
          </contact>
          <max_transient_velocity>0.01</max_transient_velocity>
        </simbody>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <gravity>0 0 -9.8</gravity>
    </world>
    



```python
# A world can contain multiple models, add an empty model by using the add_model method
world.reset()
world.add_model('model_1')
print(world)
```

    <world name="default">
      <physics default="1" name="default_physics" type="ode">
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <gravity>0 0 -9.8</gravity>
      <model name="model_1"/>
    </world>
    



```python
# You can add a model to the world as well
model = create_sdf_element('model')
world.add_model('model_2', model)
print(world)
```

    <world name="default">
      <physics default="1" name="default_physics" type="ode">
        <max_contacts>20</max_contacts>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <gravity>0 0 -9.8</gravity>
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

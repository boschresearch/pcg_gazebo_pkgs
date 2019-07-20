# pcg_gazebo.simulation.physics

# Physics
```python
Physics(self, max_step_size=0.001, real_time_factor=1, real_time_update_rate=1000, max_contacts=20, engine='ode', name='default_physics', default=False)
```

# Bullet
```python
Bullet(self, max_step_size=0.001, real_time_factor=1, real_time_update_rate=1000, max_contacts=20, min_step_size=0.0001, iters=50, sor=1.3, cfm=0, erp=0.2, contact_surface_layer=0.001, split_impulse=True, split_impulse_penetration_threshold=-0.01)
```

# ODE
```python
ODE(self, max_step_size=0.001, real_time_factor=1, real_time_update_rate=1000, max_contacts=20, min_step_size=0.0001, iters=50, sor=1.3, type='quick', precon_iters=0, use_dynamic_moi_scaling=False, friction_model='pyramid_model', cfm=0, erp=0.2, contact_surface_layer=0.001, contact_max_correcting_vel=100)
```

# Simbody
```python
Simbody(self, max_step_size=0.001, real_time_factor=1, real_time_update_rate=1000, max_contacts=20, min_step_size=0.0001, accuracy=0.001, max_transient_velocity=0.01, stiffness=100000000.0, dissipation=100, plastic_coef_restitution=0.5, plastic_impact_velocity=0.5, static_friction=0.9, dynamic_friction=0.9, viscous_friction=0.9, override_impact_capture_velocity=0.001, override_stiction_transition_velocity=0.001)
```


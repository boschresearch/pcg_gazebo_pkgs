# pcg_gazebo.generators.engines
Pose generator engine definitions that compute the pose of the
models according to pre-defined rules.

## create_engine
```python
create_engine(tag, **kwargs)
```
Engine factory that returns the engine according
to its `LABEL` definition. It returns `None` if the engine name
is invalid.

> *Input parameters*

* `tag` (*type:* `str`): Name of the engine class
* `kwargs`: Inputs for the engine class constructor

# Engine
```python
Engine(self, callback_fcn_get_model, callback_fcn_get_constraint=None, models=None, constraints=None)
```
Base class for model factory engines. The engines are responsible
of computing poses for models in the world and performing collision
checks within the scenario before the final world is generated.
This class includes methods common to all derived engine classes.

> *Input arguments*

* `callback_fcn_get_model` (*type:* `callable`): Handle to a function
or a lambda function that returns a `pcg_gazebo.simulation.SimulationModel`
associated with a tag name.
* `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`):
Handle to a function or a lambda function that returns a
`pcg_gazebo.constraints.Constraint` associated with a tag name.
* `models` (*type:* `list`, *default:* `None`): List of models names as `str`
relative to the models that the engine will have as assets.
* `constraints` (*type:* `list`, *default:* `None`): List of local constraint
configurations that will be applied on to the engine's model assets.

## models
List of `str`: List of model name tags regarding
the engine's model assets

## poses
`dict`: List of fixed poses associated with model
names.

## add_local_constraint
```python
Engine.add_local_constraint(self, model_name, constraint_name)
```
Add an association of a constraint definitions with an
specific model tag to be taken into account when running
the placement engine.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the model
* `constraint_name` (*type:* `str`): Name of the constraint definition

## get_local_constraints_for_model
```python
Engine.get_local_constraints_for_model(self, model_name)
```
Return the name of the local constraint definitions for
a model.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the model

> *Returns*

List of `str`: List of constraint definition names associated
with the model. If the model has no constraints, am empty
list is returned.

## apply_local_constraints
```python
Engine.apply_local_constraints(self, model)
```
Apply spatial constraints to model. This will modify the pose
of the model if it violates the constraint.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Model
instance

> *Returns*

`pcg_gazebo.simulation.SimulationModel`: Model with modified pose.

## add_model
```python
Engine.add_model(self, model)
```
Add a model name to the list of model assets for this engine.

> *Input arguments*

* `model` (*type:* `str`): Name of the model

## set_fixed_pose_models
```python
Engine.set_fixed_pose_models(self, models)
```
Function description

> *Input arguments*

* `param` (*type:* `data_type`, *default:* `data`): Parameter description

> *Returns*

Description of return values

## run
```python
Engine.run(self)
```
This function should be implemented by the derived
classes.

# FixedPoseEngine
```python
FixedPoseEngine(self, callback_fcn_get_model, callback_fcn_get_constraint=None, models=None, poses=None, constraints=None)
```
Engine that just places models on pre-configured fixed poses. This
engine only accepts one model asset.

* `callback_fcn_get_model` (*type:* `callable`): Handle to a function
or a lambda function that returns a `pcg_gazebo.simulation.SimulationModel`
associated with a tag name.
* `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`):
Handle to a function or a lambda function that returns a
`pcg_gazebo.constraints.Constraint` associated with a tag name.
* `models` (*type:* `list`, *default:* `None`): List of models names as `str`
relative to the models that the engine will have as assets.
* `constraints` (*type:* `list`, *default:* `None`): List of local constraint
configurations that will be applied on to the engine's model assets.
* `poses` (*type:* `list`): List of 6- (position and Euler angles) or 7 element
(position and quaternion) poses.

## add_pose
```python
FixedPoseEngine.add_pose(self, pose)
```
Add pose to the list of fixed-poses.

> *Input arguments*

* `pose` (*type:* `list`): 6- (position and Euler angles) or 7 element
(position and quaternion) poses.

## run
```python
FixedPoseEngine.run(self)
```
Generate instances of the model asset for all
the poses provided. If any local constraints were also provided,
they will be applied to the model after its placement.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel`: Model instances.

# PatternEngine
```python
PatternEngine(self, callback_fcn_get_model, callback_fcn_get_constraint=None, models=None, poses=None, constraints=None, pose=[0, 0, 0, 0, 0, 0], mode=None, args=None)
```

# RandomPoseEngine
```python
RandomPoseEngine(self, callback_fcn_get_model=None, callback_fcn_get_constraint=None, is_ground_plane=False, models=None, max_num=None, no_collision=True, max_area=1, constraints=None, policies=None, model_picker='random')
```
Placement engine that generates a random pose for its model
assets respecting input local constraints, if any is provided,
such as workspace constraint. This engine performs also a
collision check with all models already placed in the scene (except
for models flagged as ground plane) to ensure no models are overlapping
each other.

> *Input arguments*

* `callback_fcn_get_model` (*type:* `callable`): Handle to a function
or a lambda function that returns a `pcg_gazebo.simulation.SimulationModel`
associated with a tag name.
* `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`):
Handle to a function or a lambda function that returns a
`pcg_gazebo.constraints.Constraint` associated with a tag name.
* `models` (*type:* `list`, *default:* `None`): List of models names as `str`
relative to the models that the engine will have as assets.
* `constraints` (*type:* `list`, *default:* `None`): List of local constraint
configurations that will be applied on to the engine's model assets.
* `max_num` (*type:* `dict`, *default:* `None`): Maximum number of instances
of the model assets, the key being the model asset's name, and the value
the maximum number.
* `no_collision` (*type:* `bool`, *default:* `True`): If `True`, the model
instances are only added to the world if there are no collisions with the
already existing models (except for models flagged as ground plane).
* `max_area` (*type:* `float`, *default:* `1`): Percentage of the allowed
area to fill with the models.
* `model_picker` (*type:* `str`, *default:* `random`): Strategy for picking
a model from the list of assets for the next placement in the world. Options
are `random` (selecting a random model from the list of assets) or `area`
(selecting the models for the biggest to the smallest).
* `policies` (*type:* `dict`, *default:* `None`): The rules for model
generation associated with each degree of freedom.

```yaml
policies:
    - models:
     - model_1
     - model_2
     - model_3
     config:
     - dofs:
       - x
       - y
       policy:
         name: workspace
         args: area_1     # For more information on workspaces, check the class definition for `pcg_gazebo.constraints.WorkspaceConstraint`
     - dofs:
       - z
       - roll
       - pitch
       policy:
         name: value
         args: 0
     - dofs:
       - yaw
       policy:
         name: uniform
         args:
            min: -3.141592653589793
            max: 3.141592653589793
```

## reset_counter
```python
RandomPoseEngine.reset_counter(self)
```
Reset all model counters.
## increase_counter
```python
RandomPoseEngine.increase_counter(self, name)
```
Increase the counter for a model.

> *Input arguments*

* `name` (*type:* `str`): Model name

## get_num_models
```python
RandomPoseEngine.get_num_models(self, name)
```
Return the current value for the model counter.

> *Input arguments*

* `name` (*type:* `str`): Model name

> *Returns*

`int`: Number of models

## get_max_num_models
```python
RandomPoseEngine.get_max_num_models(self, name)
```
Return the defined maximum number of instances for a model.

> *Input arguments*

* `name` (*type:* `str`): Model name

> *Returns*

`int`: Maximum number of instances

## choose_model
```python
RandomPoseEngine.choose_model(self)
```
Select the next model instance to be placed in the world.
This method is affected by the constructor input `model_picker`.
In case the `model_picker` option was set as `random`, a random
model will be chosen from the assets available. If it is `area`,
the models will be ordered by footprint size and the models are
chosen by an descending footprint size.

> *Returns*

`pcg_gazebo.simulation.SimulationModel`: Chosen model

## is_model_in_workspace
```python
RandomPoseEngine.is_model_in_workspace(self, footprint)
```
Verify if the model is in the allowed workspace

> *Input arguments*

* `footprint` (*type:* `dict` or `shapely.geometries.Polygon`): A `shapely`
polygon or a dictionary with the values being the footprints for different
submodels.

> *Returns*

`bool`: `True` if the polygon is entirely contained inside the workspace

## get_list_of_footprint_polygons
```python
RandomPoseEngine.get_list_of_footprint_polygons(self, footprint)
```
Return the list of polygons contained in the `footprint` input.

> *Input arguments*

* `footprint` (*type:* `dict` or `shapely.geometries.Polygon`): A `shapely`
polygon or a dictionary with the values being the footprints for different
submodels.

> *Returns*

List of `shapely.geometry.Polygon`: List of footprint polygons

## has_collision
```python
RandomPoseEngine.has_collision(self, model)
```
Run the collision checker of the input `model`
against the current scene of the simulation.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):


> *Returns*

`bool`: `True`, if any collision is detected

## run
```python
RandomPoseEngine.run(self)
```
Run the placement engine and generate a list of models placed
according to the input policies and respecting spatial constraints.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel`


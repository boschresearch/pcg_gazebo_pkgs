
# pcg_gazebo.generators
The tools in this modules allow the generation of models and worlds using
policy rules for object placement and constraints.


# pcg_gazebo.generators._collection_manager


# pcg_gazebo.generators.assets_manager


## AssetsManager
```python
AssetsManager()
```
Assets manager containing all valid Gazebo models and model group
generators. This collection should be initialized as a singleton object
in order to have a single source of model to all instances of engines, model
and world generators.
The asset types allowed to be added are:

* `pcg_gazebo.simulation.SimulationModel`: Description for a model
* `pcg_gazebo.simulation.Light`: Description for light sources
* `pcg_gazebo.simulation.ModelGroup`: Group of models and light sources
* `pcg_gazebo.generators.ModelGroupGenerator`: Dynamic model group generator
* `dict`: Input configuration of the `creators` factory methods for `box`,
`sphere`, `cylinder` and `mesh` models, for an instance of
`pcg_gazebo.simulation.Light`, or an instance of
`pcg_gazebo.generators.ModelGroupGenerator`
* `str`: Name of an existing Gazebo model that can be found in the
Gazebo resources path


### ground_planes
`list`: List of strings with tags of ground plane models

### tags
`list`: List of strings with all asset tags

### get_instance
Return singleton instance of the `AssetsMananger`

### is_model
```python
AssetsManager.is_model(tag)
```
Return if asset identified by `tag` is an instance of `pcg_gazebo.simulation.SimulationModel`.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_light
```python
AssetsManager.is_light(tag)
```
Return if asset identified by `tag` is an instance of `pcg_gazebo.simulation.Light`.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_model_group
```python
AssetsManager.is_model_group(tag)
```
Return if asset identified by `tag` is an instance of `pcg_gazebo.simulation.ModelGroup`.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_gazebo_model
```python
AssetsManager.is_gazebo_model(tag)
```
Return if asset identified by `tag` is a Gazebo model
found in Gazebo's resources path.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_model_group_generator
```python
AssetsManager.is_model_group_generator(tag)
```
Return if asset identified by `tag` is an instance of
`pcg_gazebo.generators.ModelGroupGenerator`.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_ground_plane
```python
AssetsManager.is_ground_plane(tag)
```
Return if asset identified by `tag` is flagged as a ground
plane model.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### is_factory_input
```python
AssetsManager.is_factory_input(tag)
```
Return if asset identified by `tag` is a `dict` containing
the inputs for a `pcg_gazebo.generators.creators` factory method
to create a `box`, `sphere`, `cylinder` or `mesh` model.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset.


### add
```python
AssetsManager.add(description,
                  tag=None,
                  type=None,
                  parameters=None,
                  include_dir=None)
```
Add new asset to the collection.

> *Input arguments*

* `description` (*type:* `str`, `dict`, `pcg_gazebo.simulation.SimulationModel`,
`pcg_gazebo.simulation.Light`, `pcg_gazebo.simulation.ModelGroup` or
`pcg_gazebo.generators.ModelGroupGenerator`): Model description.
* `tag` (*type:* `str`, *default:* `None`): Asset's tag. If `None` is provided,
the input `description` must have an attribute `name` which will be used as a
tag, otherwise the function returns `False`.
* `type` (*type:* `str`, *default:* `None`): When the provided description is
`dict`, the type of asset that must be generated with the `dict` input must be
then provided as either `factory`, `model_generator` or `light`.

> *Returns*

`True`, if asset could be added to the collection.


### get
```python
AssetsManager.get(tag, *args, **kwargs)
```
Return an asset reference by `tag`.

> *Input arguments*

* `tag` (*type:* `str`): Tag of the asset. In case `tag` is referencing a
`pcg_gazebo.generators.ModelGroupGenerator`, additional inputs to run
the engines can be provided using `*args` and `**kwargs`.

> *Returns*

`pcg_gazebo.simulation.SimulationModel` or `pcg_gazebo.simulation.ModelGroup`.
`None`, if `tag` is invalid.


### set_asset_as_ground_plane
```python
AssetsManager.set_asset_as_ground_plane(tag)
```
Flag a model asset as part of the ground plane. This procedure will
affect the collision checks during the automatic placement of models in
the world using the placement engines.

> *Input arguments*

* `tag` (*type:* `str`): Name of the model asset


### from_dict
```python
AssetsManager.from_dict(config)
```
Read assets from an input `dict`. The dictionary should have a list of
asset descriptions under the tag `assets` and, if necessary, a list of
strings referring to models that must be flagged as ground plane under the
tag `ground_plane`.

> *Input arguments*

* `config` (*type:* `data_type`, *default:* `data`): Parameter description

> *Returns*

Description of return values


### from_yaml
```python
AssetsManager.from_yaml(filename)
```
Load the assets from a YAML file.

> *Input arguments*

* `filename` (*type:* `str`): YAML filename.


# pcg_gazebo.generators.constraints_manager


## ConstraintsManager
```python
ConstraintsManager()
```


### add
```python
ConstraintsManager.add(name, type, **kwargs)
```
Add a new positioning constraint class to the internal
constraints list.

> *Input arguments*

* `name` (*type:* `str`): ID name for the constraint class instance
* `type` (*type:* `str`): Name of the constraints class to be created
* `kwargs` (*type:* `dict`): Input arguments for the constraint class
to be created


# pcg_gazebo.generators.collision_checker


## CollisionChecker
```python
CollisionChecker()
```
Mesh-based collision checker manager. The meshes can be
added to the collision check scene and used to check any
other object for collisions.

> *Input arguments*

* `ignore_ground_plane` (*type:* `bool`, *value:* `True`): Ignores the meshes flagged as ground plane when performing collision checks.


### scene
`trimesh.scene.Scene`: Collision check scenario

### reset_scenario
```python
CollisionChecker.reset_scenario()
```
Remove all meshes from collision check scene.

### reset_to_fixed_model_scenario
```python
CollisionChecker.reset_to_fixed_model_scenario()
```
Remove all meshes that were not generated by a fixed-pose engine.

### add_fixed_model
```python
CollisionChecker.add_fixed_model(model)
```
Add a model as a fixed-pose model to the scene.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure


### add_model
```python
CollisionChecker.add_model(model)
```
Add model to collision checking scene.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure


### show
```python
CollisionChecker.show()
```
Display the current collision check scenario using `pyglet`.

### check_collision_with_current_scene
```python
CollisionChecker.check_collision_with_current_scene(model)
```
Check if there are any collisions between `model` and
the meshes in the scene.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model structure

> *Returns*

`True`, if any collision is detected. `False`, otherwise.


### check_for_collisions
```python
CollisionChecker.check_for_collisions()
```
Check if there are any collisions amongst the meshes in the scene.

> *Returns*

`True`, if any collision is detected. `False`, otherwise.


## SingletonCollisionChecker
```python
SingletonCollisionChecker()
```
Singleton collision checker that can be have one instance accessed by
multiple clients. It facilitates sharing the collision managar amongst
many engines, for example.

> *Attributes*

* `INSTANCE` (*type:* `SingletonCollisionChecker`, *value:* `None`): Instance
of the singleton collision checker that is initialized by the first call of
`get_instance()`.

> *Input arguments*

* `ignore_ground_plane` (*type:* `bool`, *value:* `True`): Ignores the meshes
flagged as ground plane when performing collision checks.


### get_instance
Return a singleton instance of the collision checker.

> *Input arguments*

* `kwargs` (*type:* `dict`): Input arguments for the `SingletonCollisionChecker`
instance.

> *Returns*

A `SingletonCollisionChecker` instance


# pcg_gazebo.generators.engine_manager


## EngineManager
```python
EngineManager()
```


### add
```python
EngineManager.add(tag, engine_name, models, **kwargs)
```
Add a new model creator engine to the internal engines list.

> *Input arguments*

* `engine_name` (*type:* `str`): Name of the engine class to be created
* `models` (*type:* list of `str`): Name of the models that will be assets
to the created engine
* `kwargs` (*type:* `dict`): Input arguments to the created engine.


### from_yaml
```python
EngineManager.from_yaml(filename)
```
Load the engines from a YAML file.

> *Input arguments*

* `filename` (*type:* `str`): YAML filename.


# pcg_gazebo.generators.creators
Factory methods to create simulation models.

## box
```python
box(size, mass=0, name='box', pose=[0, 0, 0, 0, 0, 0], color=None)
```
Factory method that returns a box-shaped model with one cuboid link.

> *Input arguments*

* `size` (*type:* `list` or `numpy.ndarray`): 3D vector with the size of the box for width,
length and height, respectively, in meters.
* `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
mass is not greater than zero, the model is set as static.
* `name` (*type:* `str`, *default:* `'box'`): Name of the model.
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`): Origin of the model.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model. It can be provided as a
RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

A box-shaped `pcg_gazebo.simulation.SimulationModel` instance.


## mesh
```python
mesh(visual_mesh_filename,
     collision_mesh_filename=None,
     use_approximated_collision=False,
     approximated_collision_model='box',
     visual_mesh_scale=[1, 1, 1],
     collision_mesh_scale=[1, 1, 1],
     name='mesh',
     pose=[0, 0, 0, 0, 0, 0],
     color=None,
     mass=0,
     inertia=None,
     use_approximated_inertia=True,
     approximated_inertia_model='box')
```
Create a model based on a mesh input. The options for visual and
collision meshes are:

* `visual_mesh_filename` is provided and no `collision_mesh_filename`.
The collision mesh is then set to be the same as the visual mesh.
* Both `visual_mesh_filename` and `collision_mesh_filename` are provided
separately.
* `visual_mesh_filename` is provided and no `collision_mesh_filename`, but
`use_approximated_collision` is `True`. In this case the collision geometry
can be an approximated geometry fitted to the visual mesh. Options for
the approximation methods are `box`, `sphere` or `cylinder`.

The same is valid for the moments of inertia. For static models, `mass`
can be set as 0. Otherwise, the following options are possible:

* `inertia` provided as `inertia=dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)`
* Set `use_approximated_inertia` to `True` and the inertia model will be
computed for the model using the `approximated_inertia_model` input (options
are `box`, `sphere` or `cylinder`). In this case the approximated geometry
will be computed from the visual mesh and its dimensions combined with
the provided `mass` will be used to generate the model's moments of inertia.

> *Input arguments*

* `visual_mesh_filename` (*type:* `str`): Name of the visual mesh file
* `collision_mesh_filename` (*type:* `str`, *default:* `None`): Name of the
collision mesh file. If `None` is provided, then the visual mesh file will
be used as collision geometry
* `use_approximated_collision` (*type:* `bool`, *default:* `False`): Enable
computing an approximated collision geometry from the visual mesh.
* `approximated_collision_model` (*type:* `str`, *default:* `'box'`): Type
of approximated collision geometry to be derived from the visual mesh. Options
are `box`, `cylinder` or `sphere`.
* `visual_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`): Scaling vector
to be applied to the visual mesh
* `collision_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`): Scaling vector
to be applied to the collision mesh
* `name` (*type:* `str`, *default:* `'box'`): Name of the model.
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`):
Origin of the model.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model. It
can be provided as a RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/)
or a specific `xkcd` color name, and/or `random` for a random RGBA color.
* `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
mass is not greater than zero, the model is set as static.
* `inertia` (*type:* `dict`, *default:* `None`): Optional moments of inertia
setting to the model in the form of `inertia=dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)`
* `use_approximated_inertia` (*type:* `bool`, *default:* `True`): Enable
computation of moments of inertia based on the `mass` input and the approximated
inertia model setting based on the dimensions of the mesh.
* `approximated_inertia_model` (*type:* `str`, *default:* `box`): Type of
geometrical approximation to be computed from the visual mesh. The dimensions
of the approximated geometry will be then used to compute the moments of inertia of
the model. Options are `box`, `cylinder` or `sphere`.

> *Returns*

A box-shaped `pcg_gazebo.simulation.SimulationModel` instance.


## sphere
```python
sphere(radius, mass=0, name='sphere', pose=[0, 0, 0, 0, 0, 0], color=None)
```
Return a sphere-shaped simulation model.

> *Input arguments*

* `radius` (*type:* `float`): Radius of the sphere.
* `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
mass is not greater than zero, the model is set as static.
* `name` (*type:* `str`, *default:* `'box'`): Name of the model.
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`): Origin of the model.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model. It can be provided as a
RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

A sphere-shaped `pcg_gazebo.simulation.SimulationModel` instance.


## cylinder
```python
cylinder(length,
         radius,
         mass=0,
         name='cylinder',
         pose=[0, 0, 0, 0, 0, 0],
         color=None)
```
Return a cylinder-shaped simulation model with the rotation axis
set per default as `[0, 0, 1]`.

> *Input arguments*

* `radius` (*type:* `float`): Radius of the cylinder.
* `length` (*type:* `float`): Length of the cylinder.
* `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
mass is not greater than zero, the model is set as static.
* `name` (*type:* `str`, *default:* `'box'`): Name of the model.
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`): Origin of the model.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model. It can be provided as a
RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

A cylinder-shaped `pcg_gazebo.simulation.SimulationModel` instance.


## box_factory
```python
box_factory(size,
            mass=None,
            name='box',
            pose=[0, 0, 0, 0, 0, 0],
            use_permutation=True,
            color=None)
```
Factory function for box-shaped models. It parses the vector `size`
to generate the boxes. The `mass` can be either a scalar or a vector.
If `mass` is a scalar, all boxes will have the same mass. If the size
of the vectors `size` and `mass` are the same, the boxes can be generated
by associating a `size` vector with a mass by position in the array or they
can be permutated. If the vectors `size` and `mass` have different lengths,
only permutation can be performed.

The `size` and `mass` inputs can also be provided as lambda functions
as `str`, such as:

```python
size="__import__('numpy').random.random((4, 3))"
mass="__import__('numpy').arange(1, 10, 4)"
```

> *Input arguments*

* `size` (*type:* `list` or lambda function as `str`): List of 3D size vectors
* `mass` (*type:* `float`,  list of `float` or lambda function as `str`,
*default:* `None`): Mass of the boxes. If `mass` is `None`, all boxes
will be static models
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`):
Origin of the model.
* `use_permutation` (*type:* `bool`, *default:* `True`): Enable use of
permutation to associate the `size` elements with the `mass` inputs. If the
sizes of the `size` and `mass` have different sizes, permutation will be used
per default.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model.
It can be provided as a RGBA vector, `xkcd` for a random
[XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel` instances.


## sphere_factory
```python
sphere_factory(radius,
               mass=None,
               name='sphere',
               pose=[0, 0, 0, 0, 0, 0],
               use_permutation=True,
               color=None)
```
Factory function for sphere-shaped models. It parses the vector `radius`
to generate the spheres. The `mass` can be either a scalar or a vector.
If `mass` is a scalar, all spheres will have the same mass. If the size
of the vectors `radius` and `mass` are the same, the spheres can be generated
by associating a `radius` value with a mass by position in the array or they
can be permutated. If the vectors `radius` and `mass` have different lengths,
only permutation can be performed.

The `radius` and `mass` inputs can also be provided as lambda functions
as `str`, such as:

```python
radius="__import__('numpy').random.random(2)"
mass="__import__('numpy').arange(1, 4, 1)"
```

> *Input arguments*

* `radius` (*type:* `list` or lambda function as `str`): List of radius values
* `mass` (*type:* `float`, list of `float` or lambda function as `str`,
*default:* `None`): Mass of the boxes. If `mass` is `None`, all spheres will
be static models
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`):
Origin of the model.
* `use_permutation` (*type:* `bool`, *default:* `True`): Enable use of
permutation to associate the `radius` elements with the `mass` inputs. If the
sizes of the `radius` and `mass` have different sizes, permutation will be used
per default.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model.
It can be provided as a RGBA vector, `xkcd` for a random
[XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel` instances.


## cylinder_factory
```python
cylinder_factory(length,
                 radius,
                 mass=None,
                 name='cylinder',
                 pose=[0, 0, 0, 0, 0, 0],
                 use_permutation=True,
                 color=None)
```
Factory function for cylinder-shaped models. It parses the vectors `radius`
and `length` to generate the cylinders. The `mass` can be either a scalar or a
vector. If `mass` is a scalar, all cylinders will have the same mass. If the
size of the vectors `length`, `radius` and `mass` are the same, the cylinders
can be generated by associating a `radius` and a `length` value with a mass by
position in the array or they can be permutated. If the vectors `radius` and
`length` have different lengths, only permutation can be performed.

The `length`, `radius` and `mass` inputs can also be provided as lambda
functions as `str`, such as:

```python
length="__import__('numpy').random.random(2)"
radius="__import__('numpy').random.random(2)"
mass="__import__('numpy').arange(1, 4, 1)"
```

> *Input arguments*

* `radius` (*type:* `float`, list of `float` or lambda function as `str`):
List of radius values
* `length` (*type:* `float`, list of `float` or lambda function as `str`):
List of length values
* `mass` (*type:* `float`, list of `float` or lambda function as `str`,
*default:* `None`): Mass of the cylinders. If `mass` is `None`, all
cylinders will be static models
* `pose` (*type:* `list` or `numpy.array`, *default:* `[0, 0, 0, 0, 0, 0]`):
Origin of the model.
* `use_permutation` (*type:* `bool`, *default:* `True`): Enable use of
permutation to associate the `size` elements with the `mass` inputs. If the
sizes of the `length` and `radius` have different sizes, permutation will
be used per default.
* `color` (*type:* `str` or `list`, *default:* `None`): Color of the model.
It can be provided as a RGBA vector, `xkcd` for a random
[XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
color name, and/or `random` for a random RGBA color.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel` instances.


## config2models
```python
config2models(config)
```
Parse the input `dict` configuration and calls the respective
model factory.

> *Input arguments*

* `config` (*type:* `dict`): Dictionary with the model generation
rules

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel` instances.


## create_models_from_config
```python
create_models_from_config(config, n_processes=None)
```
Creation of models from a `dict` configuration input using
multi-processing.

> *Input arguments*

* `config` (*type:* `dict`): Dictionary with the model generation
rules
* `n_processes` (*type:* `int`, *default:* `None`): Maximum number of
processes. If `None`, then use the number of CPUs available.

> *Returns*

List of `pcg_gazebo.simulation.SimulationModel` instances.


# pcg_gazebo.generators.model_group_generator


# pcg_gazebo.generators.occupancy


# pcg_gazebo.generators.patterns


# pcg_gazebo.generators.world_generator


## WorldGenerator
```python
WorldGenerator()
```
Generation of full Gazebo worlds, including physics engine configuration,
modes and lights.

> *Input arguments*

* `gazebo_proxy` (*type:* `pcg_gazebo.task_manager.GazeboProxy`,
*default:* `None`): A `GazeboProxy` object to enable spawning of models
and configuration of the simulation in runtime.



### assets
List of `pcg_gazebo.simulation.SimulationModel`: List of model
assets that will be used of the world generation.


### constraints
`dict` of `pcg_gazebo.generators.constraints`: Dictionary with the
positioning constraints.


### engines
`dict` of `pcg_gazebo.generators.engines`: Dictionary with the
model creation engines.


### gazebo_proxy
`pcg_gazebo.task_manager.GazeboProxy`: Internal instance
of the `GazeboProxy`


### name
`str`: Name of the generated world

### world
`pcg_gazebo.simulation.World`: World abstraction
instance


### init_gazebo_proxy
```python
WorldGenerator.init_gazebo_proxy(ros_host='localhost',
                                 ros_port=11311,
                                 gazebo_host='localhost',
                                 gazebo_port=11345,
                                 timeout=30,
                                 ignore_services=None)
```
Initialize a `GazeboProxy` instance to interface with a running
instance of Gazebo. If a `GazeboProxy` already exists, it will be
deleted before a new one is created.

> *Input arguments*

* `ros_host` (*type:* `str`, *default:* `localhost`): Address of the
ROS host machine running `roscore`.
* `ros_port` (*type:* `int`, *default:* `11311`): Port number for
`roscore`
* `gazebo_host` (*type:* `str`, *default:* `localhost`): Address of the
Gazebo server
* `gazebo_port` (*type:* `int`, *default:* `11345`): Port number of
the Gazebo server


### add_engine
```python
WorldGenerator.add_engine(tag, engine_name, models, **kwargs)
```
Add a new model creator engine to the internal engines list.

> *Input arguments*

* `engine_name` (*type:* `str`): Name of the engine class to be created
* `models` (*type:* list of `str`): Name of the models that will be assets
to the created engine
* `kwargs` (*type:* `dict`): Input arguments to the created engine.


### add_constraint
```python
WorldGenerator.add_constraint(name, type, **kwargs)
```
Add a new positioning constraint class to the internal
constraints list.

> *Input arguments*

* `name` (*type:* `str`): ID name for the constraint class instance
* `type` (*type:* `str`): Name of the constraints class to be created
* `kwargs` (*type:* `dict`): Input arguments for the constraint class
to be created


### add_asset
```python
WorldGenerator.add_asset(*args, **kwargs)
```
Add a new model asset that can be used by the engines and
added to the generated world.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model


### set_model_as_ground_plane
```python
WorldGenerator.set_model_as_ground_plane(model_name)
```
Flag a model asset as part of the ground plane. This procedure will
affect the collision checks during the automatic placement of models in
the world using the placement engines.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the model asset


### get_asset
```python
WorldGenerator.get_asset(name)
```
Return a simulation model asset.

> *Input arguments*

* `name` (*type:* `str`): Name of the model asset.

> *Returns*

The model asset as `pcg_gazebo.simulation.SimulationModel`.
`None` if `name` cannot be found in the list of model assets.


### get_constraint
```python
WorldGenerator.get_constraint(name)
```
Return a positioning constraint configuration.

> *Input arguments*

* `param` (*type:* `data_type`, *default:* `data`): Parameter description

> *Returns*

Description of return values


### add_gazebo_model_as_asset
```python
WorldGenerator.add_gazebo_model_as_asset(gazebo_model_name)
```
Create a model asset by importing a Gazebo model that already
exists in the resources path of the catkin workspace. The model's
SDF file will be parsed and converted into a `pcg_gazebo.simulation.SimulationModel`
instance.

Models that include lights can also be added, but will not be considered
assets, they will just be included into the generated world SDF file.

> *Input arguments*

* `gazebo_model_name` (*type:* `str`): ID name from the Gazebo model
to be imported

> *Returns*

`True` if Gazebo model could be included in the assets list.


### is_asset
```python
WorldGenerator.is_asset(name)
```
Return `True` if the model identified by the string `name`
is part of the list of assets.

> *Input arguments*

* `name` (*type:* `str`): Name of the model


### add_model
```python
WorldGenerator.add_model(model, poses)
```
Add an instance of `pcg_gazebo.simulation.SimulationModel` to
the world in designed poses.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Parameter description
* `poses` (*type:* `list`): List of 6D pose vectors


### add_gazebo_model
```python
WorldGenerator.add_gazebo_model(model_name, pose=[0, 0, 0, 0, 0, 0])
```
Add an existent Gazebo model to the world in designed poses.

> *Input arguments*

* `model_name` (*type:* `str`): ID name of the Gazebo model
* `pose` (*type:* `list`): 6D pose vector


### remove_asset
```python
WorldGenerator.remove_asset(name)
```
Remove model asset from the list of assets.

> *Input arguments*

* `name` (*type:* `str`): Name of the model

> *Returns*

`True`, if model could be removed.


### delete_model
```python
WorldGenerator.delete_model(model_name)
```
Delete a model from the currently running Gazebo instance

> *Input arguments*

* `model_name` (*type:* `str`): Name of the model

> *Returns*

`True` if the model could be deleted from the simulation.


### add_lights_from_gazebo_model
```python
WorldGenerator.add_lights_from_gazebo_model(model_name)
```
Add light models to the generated world from a Gazebo model.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the Gazebo model

> *Returns*

`True` if the lights could be parsed and added to the world.


### from_dict
```python
WorldGenerator.from_dict(config)
```
Parse a configuration settings `dict` with all information on the
list of model assets, engines, constraints and lights and instantiate the
necessary objects.

An example of a YAML file that can hold this kind of information can be
seen below:

```yaml
name: world_name
assets:
- model_1       # This list holds only Gazebo models
- model_2
- model_3
ground_plane:   # Optional input
- model_1       # If model_1 is part of the ground_plane, it should be flagged for collision checking
constraints:
- name: kitchen                     # Name identifier
    type: workspace                 # Name of the constraint class
    frame: world
    geometry:
    type: area
    description:
        points:
        - [-6.54833, -4.17127, 0]
        - [-3.24447, -4.17127, 0]
        - [-3.24447, 0.12423, 0]
        - [-6.54833, 0.12423, 0]
- name: tangent_to_ground_plane     # Name identifier
    type: tangent                   # Name of the constraint class
    frame: world
    reference:
    type: plane
    args:
        origin: [0, 0, 0]
        normal: [0, 0, 1]
engines:
- engine_name: fixed_pose
  models:
  - sll_room_empty
  poses:
  - [0, 0, 0, 0, 0, 0]
- engine_name: random_pose
  models:
  - sll_table_group_futura_seat
  - sll_table_group_futura
  model_picker: size
  max_area: 0.9
  no_collision: false
  max_num:
    sll_table_group_futura_seat: 6
    sll_table_group_futura: 1
  policies:
    - models:
        - sll_table_group_futura_seat
        - sll_table_group_futura
        config:
        - dofs:
        - x
        - y
        policy:
            name: workspace
            args: dining_room
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
            mean: 0
            min: -3.141592653589793
            max: 3.141592653589793
  constraints:
    - model: sll_table_group_futura
        constraint: tangent_to_ground_plane
    - model: sll_table_group_futura_seat
        constraint: tangent_to_ground_plane
lights:
- name: sun     # Name of the Gazebo model with the light data
```

> *Input arguments*

* `config` (*type:* `dict`): Configuration settings for the world generator

> *Returns*

Description of return values


### spawn_model
```python
WorldGenerator.spawn_model(model,
                           robot_namespace,
                           pos=[0, 0, 0],
                           rot=[0, 0, 0],
                           reference_frame='world',
                           timeout=30,
                           replace=False)
```
Spawn a `pcg_gazebo.simulation.SimulationModel` in a running instance
of Gazebo. A `GazeboProxy` is required for this method to finish successfully.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Simulation model
to be spawned
* `robot_namespace` (*type:* `str`): Name under which the robot should be
spawned in Gazebo
* `pos` (*type:* `list`, *default:* `[0, 0, 0]`): Spawning position wrt reference
frame
* `rot` (*type:* `list`, *default:* `[0, 0, 0]`): Roll-Pitch-Yaw angles in radians
or a (w, i, j, k) quaternion vector.
* `reference_frame` (*type:* `str`, *default:* `world`): Reference frame for the
spawning pose
* `timeout` (*type:* `float`): Timeout in seconds to wait for Gazebo to start
* `replace` (*type:* `bool`, *default:* `False`): Replace the model in the simulation
in case a model with the same name already exists.

> *Returns*

`True` if the model could be spawned.


### get_physics_engine
```python
WorldGenerator.get_physics_engine(engine='ode')
```
Return an instance of a physics engine as
`pcg_gazebo.simulation.physics.Physics` object.

> *Input arguments*

* `engine` (*type:* `str`): ID name of the physics
engine, options are `ode`, `bullet` and `simbody`.

> *Returns*

An `pcg_gazebo.simulation.physics.Physics` object.


### run_engines
```python
WorldGenerator.run_engines(attach_models=False)
```
Run all the model placement engines and add the generated
models in the internal instance of the world representation.

> *Input arguments*

* `attach_models` (*type:* `bool`, *default:* `False`): Attach
the generated models to the existent list of models in the world

> *Returns*

`True` if all engines ran successfully.


### reset_world
```python
WorldGenerator.reset_world(name, engine='ode', gravity=[0, 0, -9.8])
```
Reset the generated world instance to its default state and
without any models.

> *Input arguments*

* `name` (*type:* `str`): Name of the world
* `engine` (*type:* `str`, *default:* `ode`): Name of the
physics engine to be used. Options are `ode`, `bullet` or `simbody`.
* `gravity` (*type:* `list`, *default:* `[0, 0, -9.8]`): Gravitational
acceleration vector


### export_world
```python
WorldGenerator.export_world(output_dir=None,
                            filename=None,
                            with_default_ground_plane=True,
                            with_default_sun=True)
```
Export world to an SDF file that can be used by Gazebo.

> *Input arguments*

* `output_dir` (*type:* `str`, *default:* `None`): Path
to output directory to store the world file.
* `filename` (*type:* `str`, *default:* `None`): Name of the
SDF world file
* `with_default_ground_plane` (*type:* `bool`, *default:*
`True`): Add the default ground plane model to the world before
exporting it
* `with_default_sun` (*type:* `bool`, *default:* `True`): Add
the default sun model to the world before exporting it

> *Returns*

Full name of the exported SDF world file as a `str`


### plot_results
```python
WorldGenerator.plot_results(fig=None,
                            fig_width=1000,
                            fig_height=800,
                            footprint_geometry='collision',
                            engine='bokeh')
```
Plot the footprints of models included in the current world
instance.

> *Input arguments*

* `fig` (*type:* a `bokeh` or a `matplotlib` figure, *default:* `None`):
A figure object. If `fig` is `None`, a new figure will be created
* `fig_width` (*type:* `int`, *default:* `1000`): Width of the figure
* `param` (*type:* `data_type`, *default:* `data`): Parameter description

> *Returns*

Description of return values


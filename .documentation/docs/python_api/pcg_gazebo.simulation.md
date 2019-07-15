
# pcg_gazebo.simulation
Simulation interface module, with abstraction classes for all relevant
entities that form a simulation in Gazebo.


## create_object
```python
create_object(tag, **kwargs)
```
Factory method for `Link` subclasses.

> *Input arguments*

* `tag` (*type:* `str`): Name identifier of the object class
* `kwargs` (*type:* `dict`): Input arguments for the object class

> *Returns*

`Link`: Subclass instance.


## get_gazebo_model_folders
```python
get_gazebo_model_folders(dir_path)
```
Return the paths to all Gazebo model folders under the
directory `dir_path`.

> *Input arguments*

* `dir_path` (*type:* `str`): Path to the search directory.

> *Returns*

`dict`: Gazebo model paths ordered according to the
Gazebo model names.


## load_gazebo_models
```python
load_gazebo_models()
```
Search for Gazebo models in the local `.gazebo/models` folder
and in the ROS paths.

> *Returns*

`dict`: Information of all Gazebo models found


## get_gazebo_models
```python
get_gazebo_models()
```
Return the information of all Gazebo models found in the
local `.gazebo/models` folder and in the catkin workspace as
a dictionary.


## get_gazebo_model_names
```python
get_gazebo_model_names()
```
Return the names of all Gazebo models that can be found
is the local `.gazebo/models` folders and catkin workspace.


## is_gazebo_model
```python
is_gazebo_model(name)
```
Test if a model with the identifier `name` is a Gazebo
model that is found in the resources path.

> *Input arguments*

* `name` (*type:* `str`): Name identifier of the model

> *Returns*

`True` if `name` refers to a Gazebo model.


## get_gazebo_model_path
```python
get_gazebo_model_path(model_name)
```
Return the path of the Gazebo model.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the Gazebo model

> *Returns*

`str`: Path of the Gazebo model folder


## get_gazebo_model_sdf
```python
get_gazebo_model_sdf(model_name, sdf_file='model.sdf')
```
Parse the Gazebo model's SDF file into a `pcg_gazebo`
SDF instance.

> *Input arguments*

* `model_name` (*type:* `str`): Name of the Gazebo model.
* `sdf_file` (*type:* `str`, *default:* `model.sdf`): Name
of the SDF file to be parsed.

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` instance as an SDF element.


# Box
```python
Box()
```
Class derived from `pcg_gazebo.simulation.Link` to
describe a box-shaped link or single-link model.

> *Input arguments*

* `name` (*type:* `str`, *default:* `box`): Name of the object
* `size` (*type:* `list`, *default:* `[1, 1, 1]`): Vector with
width, length and height of the box,


## collision
`pcg_gazebo.simulation.properties.Collision`:
Return single box-shaped collision model.


## size
List of `float`: Size of the box as `[width, length, height]`

## visual
`pcg_gazebo.simulation.properties.Visual`:
Return single box-shaped visual model.


## to_sdf
```python
Box.to_sdf(type='model', name='box', sdf_version='1.6')
```
Convert object to an SDF element. The object can be converted
to different SDF elements according to the `type` input

* `box`: SDF box element
* `geometry`: SDF geometry element with nested element
* `collision`: SDF collision element
* `visual`: SDF visual element
* `link`: SDF link element with collision and visual properties
* `model`: single-link SDF model element
* `sdf`: SDF file format with a nested model element.

> *Input arguments*

* `type` (*type:* `str`): Type of output SDF element, options are
`collision`, `visual`, `link`, `model`, `sdf`.
* `name` (*type:* `str`, *default:* `model`): Name of the output object
* `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the output
SDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase`: SDF element instance.


## add_inertial
```python
Box.add_inertial(mass)
```
Initialize mass and moments of inertia for box model.

> *Input arguments*

* `mass` (*type:* `float`): Mass in kilograms


## update_inertial
```python
Box.update_inertial(mass=None)
```
Update mass and moments of inertia for box model.

> *Input arguments*

* `mass` (*type:* `float`): Mass in kilograms


## update_collision
```python
Box.update_collision()
```
Update collision model according to the current
`size`.


## update_visual
```python
Box.update_visual()
```
Update visual model according to the current
`size`.


# Cylinder
```python
Cylinder()
```
Class derived from `pcg_gazebo.simulation.Link` to
describe a cylinder-shaped link or single-link model.

> *Input arguments*

* `name` (*type:* `str`, *default:* `cylinder`): Name of the object.
* `length` (*type:* `float`, *default:* `1`): Length of the
cylinder in meters.
* `radius` (*type:* `float`, *default:* `1`): Radius of the
cylinder in meters.


## collision
`pcg_gazebo.simulation.properties.Collision`:
Return single cylinder-shaped collision model.


## length
`float`: Length of the cylinder in meters

## radius
`float`: Radius of the cylinder in meters

## visual
`pcg_gazebo.simulation.properties.Visual`:
Return single cylinder-shaped visual model.


## to_sdf
```python
Cylinder.to_sdf(type='model', name='cylinder', sdf_version='1.6')
```
Convert object to an SDF element. The object can be converted
to different SDF elements according to the `type` input

* `cylinder`: SDF cylinder element
* `geometry`: SDF geometry element with nested element
* `collision`: SDF collision element
* `visual`: SDF visual element
* `link`: SDF link element with collision and visual properties
* `model`: single-link SDF model element
* `sdf`: SDF file format with a nested model element.

> *Input arguments*

* `type` (*type:* `str`): Type of output SDF element, options are
`collision`, `visual`, `link`, `model`, `sdf`.
* `name` (*type:* `str`, *default:* `model`): Name of the output object
* `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the output
SDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase`: SDF element instance.


## update_inertial
```python
Cylinder.update_inertial(mass=None)
```
Initialize mass and moments of inertia for cylinder model.

> *Input arguments*

* `mass` (*type:* `float`): Mass in kilograms


## update_collision
```python
Cylinder.update_collision()
```
Update collision model according to the current
`length` and `radius`.


## update_visual
```python
Cylinder.update_visual()
```
Update visual model according to the current
`length` and `radius`.


# Joint
```python
Joint()
```


# Light
```python
Light()
```


# SimulationModel
```python
SimulationModel()
```


# Link
```python
Link()
```
Representation of a simulated `link` or a single-link `model`.

> *Input arguments*

* `name` (*type:* `str`, *value:* `object`): Name of the object.
* `creation_time` (*type:* `float`, *default:* `None`): Timestamp
of the creation of the object in Gazebo.
* `life_timeout` (*type:* `float`, *default:* `None`): Timeout in which
to remove the object from the simulation (**not implemented**).


## collisions
List of `pcg_gazebo.simulation.properties.Collision`:
List of collision models


## creation_time
`float`: Time of creation of this object, if
it represents a single-link model.


## inertial
`pcg_gazebo.simulation.properties.Inertial`:
Description of the object's moments of inertia.


## kinematic
`bool`: Flag to indicate if the model is purely kinematic

## life_timeout
`float`: Life timeout timestamp for this object,
if it represents a single-link model


## name
`str`: Object name

## pose
`pcg_gazebo.simulation.properties.Pose`: Pose of the object

## self_collide
`bool`: Self-collision flag

## static
`bool`: Flag to indicate if object is static

## visuals
List of `pcg_gazebo.simulation.properties.Visual`:
List of visual models


## create_link_from_mesh
Factory method to build a link or single-link model from a mesh.
This method allows not only assigning a mesh as a visual and collision
geometry, but also using geometrical approximations of the input mesh
to create, for example, a collision mesh, or computing the moments of
inertia.

> *Input arguments*

* `name` (*type:* `str`, *default:* `link`): Name of the link.
* `visual_mesh_filename` (*type:* `str`, *default:* `None`): Filename
to the visual mesh file.
* `collision_mesh_filename` (*type:* `str`, *default:* `None`): Filename
to the collision mesh file. If the input is `None` and `use_approximated_collision` is
`False`, the visual mesh will be also set as collision mesh.
* `use_approximated_collision` (*type:* `bool`, *default:* `False`): If `True`,
the collision geometry will be approximated from the visual mesh geometry
into a model given by the `approximated_collision_model` input.
* `approximated_collision_model` (*type:* `str`, *default:* `box`): Name of the
geometry to which the visual geometry will be approximated to generated the collision
mesh, options are `box`, `cylinder` or `sphere`.
* `visual_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`): Scaling factors
for the visual mesh in X, Y and Z directions.
* `collision_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`): Scaling factors
for the collision mesh in X, Y and Z directions.
* `pose` (*type:* `list`, *default:* `[0, 0, 0, 0, 0, 0]`): Link's pose
with respect to the model frame.
* `color` (*type:* `list` or `str`, *default:* `None`): Color set to the visual
mesh. If `None` is provided, no color is set and the mesh will inherit the
material of the mesh file. If the input is `random`, a random RGB color is generated.
This input can also be set as `xkcd` for a random [`xkcd` color](https://xkcd.com/color/rgb/)
name, or a string with the name of a specific `xkcd` color (e.g., `teal`).
Otherwise, the input can be an RGB vector as a `list`.
* `mass` (*type:* `float`, *default:* `0`): Mass of the link in kilograms. If the mass
is not greater than zero, the link will be set as static.
* `inertia` (*type:* `dict`, *default:* `None`): Moments of inertia of the link. This input
can be either a dictionary defined as `dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)` or
`None`. If `None` is provided, `use_approximated_inertia` is `True` and `mass` is greater
than zero, the moments of inertia will be computed from an approximated visual mesh geometry
given by the input `approximated_inertia_model`.
* `use_approximated_inertia` (*type:* `bool`, *default:* `True`): If `True` and `mass` is
greater tha zero, the moments of inertia of the link will be computed from a approximated
visual mesh model described by `approximated_inertia_model`.
* `approximated_inertia_model` (*type:* `str`, *default:* `box`): Type of geometry
approximation to be applied to the visual geometry. The dimensions of the geometry
will then be used to compute the moments of inertia. Options are `box`, `cylinder` or
`sphere`.

> *Returns*

`pcg_gazebo.simulation.Link` instance.


## enable_collision
```python
Link.enable_collision()
```
Enable the inclusion of the collision models
in the exported SDF description.


## disable_collision
```python
Link.disable_collision()
```
Disable the inclusion of the collision models
in the exported SDF description.


## enable_visual
```python
Link.enable_visual()
```
Enable the inclusion of the visual models
in the exported SDF description.


## disable_visual
```python
Link.disable_visual()
```
Disable the inclusion of the collision models
in the exported SDF description.


## get_collision_by_name
```python
Link.get_collision_by_name(name)
```
Return the collision model associated with the input
name identifier.

> *Input arguments*

* `name` (*type:* `str`): Name of the collision model.

> *Returns*

`pcg_gazebo.simulation.properties.Collision`, or `None`
if not collision with the given name is found.


## has_collision
```python
Link.has_collision(name)
```
Test if a collision with the input name exists.

> *Input arguments*

* `name` (*type:* `str`): Name of the collision model

> *Returns*

`bool`: `True`, if a collision model exists, `False, otherwise.


## get_visual_by_name
```python
Link.get_visual_by_name(name)
```
Return the visual model associated with the input
name identifier.

> *Input arguments*

* `name` (*type:* `str`): Name of the visual model.

> *Returns*

`pcg_gazebo.simulation.properties.Visual`, or `None`
if not visual with the given name is found.


## has_visual
```python
Link.has_visual(name)
```
Test if a visual with the input name exists.

> *Input arguments*

* `name` (*type:* `str`): Name of the visual model

> *Returns*

`bool`: `True`, if a visual model exists, `False, otherwise.


## add_empty_visual
```python
Link.add_empty_visual(name='visual')
```
Create an empty visual model and add it to the object.

> *Input arguments*

* `name` (*type:* `str`, *default:* `visual`): Name of the visual
model.

> *Returns*

`bool`: `True` if visual model could be created and added to the object.
`False` if another visual with the same name already exists.


## add_visual
```python
Link.add_visual(visual)
```
Add visual model to the object. If a visual element
with the same name already exists, a suffix will be added
to the name in the format `_i`, `i` being an integer.

> *Input arguments*

* `visual` (*type:* `pcg_gazebo.simulation.properties.Visual`):
Visual element

> *Returns*

`bool`: `True`, if visual element could be added to object.


## add_empty_collision
```python
Link.add_empty_collision(name='collision')
```
Create an empty collision model and add it to the object.

> *Input arguments*

* `name` (*type:* `str`, *default:* `collision`): Name of the collision
model.

> *Returns*

`bool`: `True` if collision model could be created and added to the object.
`False` if another collision with the same name already exists.


## add_collision
```python
Link.add_collision(collision)
```
Add collision model to the object. If a collision element
with the same name already exists, a suffix will be added
to the name in the format `_i`, `i` being an integer.

> *Input arguments*

* `collision` (*type:* `pcg_gazebo.simulation.properties.Collision`):
Collision element

> *Returns*

`bool`: `True`, if collision element could be added to object.


## to_sdf
```python
Link.to_sdf(type='link', name='model', sdf_version='1.6')
```
Convert object to an SDF element. The object can be converted
to different SDF elements according to the `type` input

* `collision`: SDF collision element
* `visual`: SDF visual element
* `link`: SDF link element with collision and visual properties
* `model`: single-link SDF model element
* `sdf`: SDF file format with a nested model element.

> *Input arguments*

* `type` (*type:* `str`): Type of output SDF element, options are
`collision`, `visual`, `link`, `model`, `sdf`.
* `name` (*type:* `str`, *default:* `model`): Name of the output object
* `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the output
SDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase`: SDF element instance.


## from_sdf
Factory method to generate a `pcg_gazebo.simulation.Link` instance
from an SDF instance. Only links can be parsed.

> *Input arguments*

* `sdf` (*type:* `pcg_gazebo.parsers.sdf.Link`): SDF object

> *Returns*

`pcg_gazebo.simulation.Link`: Simulation object instance


## export_to_gazebo_model
```python
Link.export_to_gazebo_model(output_dir,
                            name='model',
                            sdf_version='1.6',
                            version='0.1.0',
                            author_names=None,
                            author_emails=None,
                            description='',
                            generate_sdf_with_version=False)
```
Export the object as a Gazebo model, in the format

```
model_dir/
    model.sdf
    model.config
```

> *Input arguments*

* `output_dir` (*type:* `str`): Name of the directory where the model
directory will be stored.
* `name` (*type:* `str`, *default:* `model`): Name of the model
* `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the SDF format
* `version` (*type:* `str`, *default:* `0.1.0`): Gazebo model version
* `author_names` (*type:* `list`, *default:* `None`): List of authors
* `author_emails` (*type:* `list`, *default:* `None`): List of e-mails
* `description` (*type:* `str`): Model description
* `generate_sdf_with_version` (*type:* `bool`, *default:* `False`): Parameter description

> *Returns*

`bool`: `True`, if Gazebo model files were exported successfully.


## add_inertial
```python
Link.add_inertial(mass)
```
This function must be implemented by derived classes.

## update_inertial
```python
Link.update_inertial()
```
This function must be implemented by derived classes.

## update_collision
```python
Link.update_collision()
```
This function must be implemented by derived classes.

## update_visual
```python
Link.update_visual()
```
This function must be implemented by derived classes.

## add_sensor
```python
Link.add_sensor(name, sensor)
```
Add sensor associated to the link.

> *Input arguments*

* `name` (*type:* `str`): Name of the sensor
* `sensor` (*type:* `pcg_gazebo.simulation.sensors.Sensor`):
Sensor description

> *Returns*

`bool`: `True`, if sensor could be added to link.


## to_markers
```python
Link.to_markers()
```
Generate `visualization_msgs/Marker` instances from the visual and/or
collision entities.

> *Returns*

`visualization_msgs/MarkerArray`


## get_footprint
```python
Link.get_footprint(mesh_type='collision',
                   pose_offset=None,
                   use_bounding_box=False,
                   z_limits=None)
```
Returns the `shapely._GEOMETRIES.Polygon` or `shapely._GEOMETRIES.MultiPolygon`
that represent the projection of the visual or collision meshes on the XY
plane.

> *Input arguments*

* `mesh_type` (*type:* `str`, *default:* `collision`): Origin of the meshes,
options are `visual` or `collision`.
* `pose_offset` (*type:* `data_type`, *default:* `None`): Pose offset to be applied
to all meshes before the footprint is computed
* `use_bounding_box` (*type:* `bool`, *default:* `False`): Use the mesh's bounding
box for the footprint calculation
* `z_limits` (*type:* `list`, *default:* `None`): Minimum and maximum limits
in the Z direction were the meshes will be sectioned.

> *Returns*

`shapely._GEOMETRIES.Polygon` or `shapely._GEOMETRIES.MultiPolygon`


## get_meshes
```python
Link.get_meshes(mesh_type='collision', pose_offset=None)
```
Return all the meshes associated with this link.

> *Input arguments*

* `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh
to be returned, options are `visual` or `collision`.
* `pose_offset` (*type:* `list`, *default:* `None`): Pose offset
to be applied to all meshes.

> *Returns*

List of `trimesh` meshes.


## get_bounds
```python
Link.get_bounds(mesh_type='collision')
```
Return the bounds of the link with respect to its meshes.

> *Input arguments*

* `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh,
options are `visual` or `collision`.

> *Returns*

`dict`: Meshes' bounds


# Plane
```python
Plane()
```


# Polyline
```python
Polyline()
```


# Sphere
```python
Sphere()
```
Class derived from `pcg_gazebo.simulation.Link` to
describe a sphere-shaped link or single-link model.

> *Input arguments*

* `name` (*type:* `str`, *default:* `sphere`): Name of the object
* `radius` (*type:* `float`, *default:* `1`): Radius of the sphere
in meters


## collision
`pcg_gazebo.simulation.properties.Collision`:
Return single sphere-shaped collision model.


## radius
`float`: Radius of the sphere in meters

## visual
`pcg_gazebo.simulation.properties.Visual`:
Return single sphere-shaped visual model.


## to_sdf
```python
Sphere.to_sdf(type='model', name='sphere', sdf_version='1.6')
```
Convert object to an SDF element. The object can be converted
to different SDF elements according to the `type` input

* `sphere`: SDF sphere element
* `geometry`: SDF geometry element with nested element
* `collision`: SDF collision element
* `visual`: SDF visual element
* `link`: SDF link element with collision and visual properties
* `model`: single-link SDF model element
* `sdf`: SDF file format with a nested model element.

> *Input arguments*

* `type` (*type:* `str`): Type of output SDF element, options are
`collision`, `visual`, `link`, `model`, `sdf`.
* `name` (*type:* `str`, *default:* `model`): Name of the output object
* `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the output
SDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase`: SDF element instance.


## add_inertial
```python
Sphere.add_inertial(mass, hollow=False)
```
Initialize mass and moments of inertia for sphere model.

> *Input arguments*

* `mass` (*type:* `float`): Mass in kilograms
* `hollow` (*type:* `bool`, *default:* `False`): Compute
moments of inertia for a hollow sphere, instead of a solid one


## update_inertial
```python
Sphere.update_inertial(mass=None)
```
Update mass and moments of inertia for sphere model.

> *Input arguments*

* `mass` (*type:* `float`): Mass in kilograms


## update_collision
```python
Sphere.update_collision()
```
Update collision model according to the current
`radius`.


## update_visual
```python
Sphere.update_visual()
```
Update visual model according to the current
`radius`.


# World
```python
World()
```
Abstraction of Gazebo's world description. This class
contains the settings configuring the world's

* physics engine
* models
* lights
* plugins
* gravity

and can be later exported into a `.world` file that Gazebo
can parse and execute.

> *Input arguments*

* `name` (*type:* `str`, *value:* `default`): Name of the world.
* `gravity` (*type:* `list`, *default:* `[0, 0, -9.8]`): Acceleration
of gravity vector.
* `engine` (*type:* `str`, *default:* `ode`): Name of the default
physics engine, options are `ode`, `bullet` or `simbody`.


## engine
`str`: Name identififier of the physics engine

## gravity
`list`: Acceleration of gravity vector

## models
`dict`: Models

## name
`str`: Name of the world

## physics
`pcg_gazebo.simulation.physics.Physics`: Physics engine instance

## reset_physics
```python
World.reset_physics(engine='ode')
```
Reset the physics engine to its default configuration.

> *Input arguments*

* `engine` (*type:* `str`, *default:* `ode`): Name identifier
of the physics engine, options are `ode`, `bullet` or `simbody`.


## reset_models
```python
World.reset_models()
```
Reset the list of models.

## add_include
```python
World.add_include(include)
```
Add a model via include method.

> *Input arguments*

* `include` (*type:* `pcg_gazebo.parsers.sdf.Include`):
SDF `<include>` element

> *Returns*

`bool`: `True`, if model directed by the `include` element
could be parsed and added to the world.


## add_model
```python
World.add_model(tag, model)
```
Add a model to the world.

> *Input arguments*

* `tag` (*type:* `str`): Model's local name in the world. If
a model with the same name already exists, the model will be
created with a counter suffix in the format `_i`, `i` being
an integer.
* `model` (*type:* `pcg_gazebo.simulaton.SimulationModel`):
Model object

> *Returns*

`bool`: `True`, if model could be added to the world.


## rm_model
```python
World.rm_model(tag)
```
Remove model from world.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the
model to be removed.

> *Returns*

`bool`: `True`, if model could be removed, `False` if
no model with name `tag` could be found in the world.


## model_exists
```python
World.model_exists(tag)
```
Test if a model with name `tag` exists in the world description.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the model.

> *Returns*

`bool`: `True`, if model exists, `False`, otherwise.


## add_plugin
```python
World.add_plugin(tag, plugin)
```
Add plugin description to the world.

> *Input arguments*

* `tag` (*type:* `str`): Name identifier for the plugin. If
a model with the same name already exists, the model will be
created with a counter suffix in the format `_i`, `i` being
an integer.
* `plugin` (*type:* `pcg_gazebo.parsers.sdf.Plugin` or
`pcg_gazebo.simulation.properties.Plugin`): Plugin description.


## rm_plugin
```python
World.rm_plugin(tag)
```
Remove plugin from world.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the
plugin to be removed.

> *Returns*

`bool`: `True`, if plugin could be removed, `False` if
no plugin with name `tag` could be found in the world.


## plugin_exists
```python
World.plugin_exists(tag)
```
Test if a plugin with name `tag` exists in the world description.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the plugin.

> *Returns*

`bool`: `True`, if plugin exists, `False`, otherwise.


## add_light
```python
World.add_light(tag, light)
```
Add light description to the world.

> *Input arguments*

* `tag` (*type:* `str`): Name identifier for the plugin. If
a model with the same name already exists, the model will be
created with a counter suffix in the format `_i`, `i` being
an integer.
* `light` (*type:* `pcg_gazebo.parsers.sdf.Light` or
`pcg_gazebo.simulation.properties.Light`): Light description


## rm_light
```python
World.rm_light(tag)
```
Remove light from world.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the
light to be removed.

> *Returns*

`bool`: `True`, if light could be removed, `False` if
no light with name `tag` could be found in the world.


## light_exists
```python
World.light_exists(tag)
```
Test if a light with name `tag` exists in the world description.

> *Input arguments*

* `tag` (*type:* `str`): Local name identifier of the light.

> *Returns*

`bool`: `True`, if light exists, `False`, otherwise.


## to_sdf
```python
World.to_sdf(type='world',
             with_default_ground_plane=True,
             with_default_sun=True)
```
Convert the world description into as `pcg_gazebo` SDF
element.

> *Input arguments*

* `type` (*type:* `str`, *default:* `world`): Type of output SDF
element to be generated, options are `world` or `sdf`. It is
important to note that to export the world description into a
file, it is necessary to have the `sdf` format.
* `with_default_ground_plane` (*type:* `bool`, *default:* `True`):
Add Gazebo's default ground plane model to the world.
* `with_default_sun` (*type:* `bool`, *default:* `True`):
Add Gazebo's default sun model to the world.

> *Returns*

`pcg_gazebo.parsers.sdf.SDF` with a world element in it or
`pcg_gazebo.parsers.sdf.World`.


## from_sdf
Parse an `pcg_gazebo.parsers.sdf.World` into a
c.

> *Input arguments*

* `sdf` (*type:* `pcg_gazebo.parsers.sdf.World`):
SDF world element

> *Returns*

`pcg_gazebo.parsers.sdf.World` instance.


## create_scene
```python
World.create_scene(mesh_type='collision', add_pseudo_color=True)
```
Return a `trimesh.Scene` with all the world's models.

> *Input arguments*

* `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh
to be included in the scene, options are `collision` or `visual`.
* `add_pseudo_color` (*type:* `bool`, *default:* `True`): If `True`,
set each mesh with a pseudo-color.


## plot_footprints
```python
World.plot_footprints(fig=None,
                      ax=None,
                      fig_width=20,
                      fig_height=20,
                      mesh_type='collision',
                      z_limits=None,
                      colormap='magma',
                      grid=True,
                      ignore_ground_plane=True,
                      line_width=1,
                      line_style='solid',
                      alpha=0.5,
                      engine='matplotlib',
                      dpi=200)
```
Plot the mesh footprint projections on the XY plane.

> *Input arguments*

* `fig` (*type:* `matplotlib.pyplot.Figure` or `bokeh.plotting.Figure` , *default:* `None`):
Figure object. If `None` is provided, the figure will be created.
* `ax` (*type:* `matplotlib.pyplot.Axes`, *default:* `None`): Axes object to add the plot.
If `None` is provided, the axes object will be created.
* `fig_width` (*type:* `float` or `int`, *default:* `20`): Width of the figure in inches,
if `engine` is `matplotlib`, or pixels, if `engine` is `bokeh`.
* `fig_height` (*type:* `float` or `int`, *default:* `20`): Height of the figure in inches,
if `engine` is `matplotlib`, or pixels, if `engine` is `bokeh`.
* `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh to consider for the
footprint computation, options are `collision` and `visual`.
* `z_limits` (*type:* `list`, *default:* `None`): List of minimum and maximum Z-levels
to consider when sectioning the meshes.
* `colormap` (*type:* `str`, *default:* `magma`): Name of the colormap to be
used. Check [this link](https://matplotlib.org/users/colormaps.html) for `matplotlib`
colormaps and [this link](https://bokeh.pydata.org/en/latest/docs/reference/palettes.html)
for `bokeh` colormaps.
* `grid` (*type:* `bool`, *default:* `True`): If `True`, add grid to the plot.
* `ignore_ground_plane` (*type:* `bool`, *default:* `True`): Ignore the models
flagged as ground plane from the plot.
* `line_width` (*type:* `float`, *default:* `1`): Width of the line of each
footprint polygon patch.
* `line_style` (*type:* `str`, *default:* `solid`): Style of the line of each
footprint polygon patch. Check this [link](https://matplotlib.org/3.1.0/gallery/lines_bars_and_markers/linestyles.html)
to see all the line style options.
* `alpha` (*type:* `float`, *default:* `0.5`): Alpha channel value for the footprint
objects.
* `engine` (*type:* `str`, *default:* `matplotlib`): Engine to use for the generation of
the figure, options are `bokeh` and `matplotlib`.
* `dpi` (*type:* `int`, *default:* `200`): Image's DPI

> *Returns*

`matplotlib.pyplot.Figure` or `bokeh.plotting.Figure`.


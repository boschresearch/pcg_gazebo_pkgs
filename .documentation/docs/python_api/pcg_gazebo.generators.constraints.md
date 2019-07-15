
# pcg_gazebo.generators.constraints
Spatial constraints for the placement of simulation entities into the world.

## create_constraint
```python
create_constraint(tag, **kwargs)
```
Constraint factory that returns the constraint according
to its `LABEL` definition. It returns `None` if the constraint name
is invalid.

> *Input parameters*

* `tag` (*type:* `str`): Name of the constraint class
* `kwargs`: Inputs for the constraint class constructor


# Constraint
```python
Constraint()
```
Abstract constraint class.

> *Attributes*

* `LABEL` (*type:* `str`): Name of the constraint class.


# TangentConstraint
```python
TangentConstraint()
```
Class that allows computation of the closes position
for a model regarding a reference to have it placed tangent
to the reference.
Reference can be a plane or another model, at the moment.

The input reference types that are supported are

* `plane`:

To set a reference plane to which models will be placed tangently,
the `reference` input must be provided as

```python
reference = dict(
    type='plane',
    args=dict(
        normal=[0, 0, 1], # A 3 element unit vector normal to the plane
        origin=[0, 0, 0]  # The 3D position of the origin of the plane
        )
)
```

> *Attributes*

* `LABEL` (*type:* `str`, *value:* `'tangent'`): Name of the constraint class
* `_REFERENCE_TYPES` (*type:* `list`): List of types of references that can be used for the computation
* `_reference` (*type:* `dict`): Arguments of the type of reference used.

> *Input arguments*

* `reference` (*type:* `dict`): Arguments for the reference used for the tangent computation
* `frame` (*type:* `str`, *default:* `world`): Name of the frame of reference with respect to which the poses are going to be generated (**not implemented**)


## apply_constraint
```python
TangentConstraint.apply_constraint(model)
```
Compute and apply the tangent constraint for the
provided model using the reference input.

> *Input arguments*

* `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Model entity to have its pose adapted so that it is placed tangent to the reference


# WorkspaceConstraint
```python
WorkspaceConstraint()
```
Class that represents the spatial workspace where models are allowed in.
The `geometry` input is a `dict` containing all the arguments necessary to
generate the workspace geometry. For now, only 2D workspaces are supported.
The `holes` input is a list of `dict` with the same geometry description of
the input `geometry` and describe exclusion areas inside the workspace.

The supported geometry inputs to represent a workspace are

* `area`

```python
geometry = dict(
    type='area'
    description=dict(
        points=[
           [0, 0, 0],
           [0, 1, 1],
           ...
           ]  # List of 3D points that describe the vertices of the plane area
    )
)
```

* `line`

```python
geometry=dict(
    type='line',
    description=dict(
        points=[
           [0, 0, 0],
           [0, 1, 1],
           ...
           ]  # List of 3D points that describe the line
    )
)
```

* `circle`

```python
geometry=dict(
    type='circle'
    description=dict(
        radius=0.0, # Radius of the circle
        center=[0, 0, 0] # Center of the circle as a 3D point
    )
)
```

**Others are still not implemented**

> *Attributes*

* `LABEL` (*type:* `str`, *value:* `workspace`): Name of the constraint class
* `GEOMETRIES` (*type:* `list`): List of input geometries that can be used to set a workspace

> *Input arguments*

* `geometry` (*type:* `dict`, *default:* `None`): Input arguments of the geometry to be generated
* `frame` (*type:* `str`, *default:* `'world'`): Name of the frame of reference of the workspace (**not implemented**)
* `holes` (*type:* `dict`, *default:* `None`): Geometries that represent exclusion areas inside the workspace


## generate_geometry
```python
WorkspaceConstraint.generate_geometry(type, description)
```
Generate a `shapely` entity according to the geometry description
provided. The input `type` containts the name of the geometry to
be generated and the necessary arguments must be provided in the `dict`
input `description`.

Possible geometries according to the different input values in `type` are:

* `area`

```python
description=dict(
   points=[
       [0, 0, 0],
       [0, 1, 1],
       ...
       ]  # List of 3D points that describe the vertices of the plane area
)
```

* `line`

```python
description=dict(
   points=[
       [0, 0, 0],
       [0, 1, 1],
       ...
       ]  # List of 3D points that describe the line
)
```

* `circle`

```python
description=dict(
  center=[-6.8, -6.8, 0] # Center of the circle
  radius=0.2  # Radius of the circle
)
```

**Others are still not implemented**

> *Input arguments*

* `type` (*type:* `str`): Geometry type. Options are: `line`, `area`, `volume`, `multi_line`, `multi_point`, `circle`
* `description` (*type:* `dict`): Arguments to describe the geometry


## get_bounds
```python
WorkspaceConstraint.get_bounds()
```
Return the polygon bounds

## get_random_position
```python
WorkspaceConstraint.get_random_position()
```
Return a random position that belongs to the workspace

## contains_point
```python
WorkspaceConstraint.contains_point(point)
```
Return True if `point` is part of the workspace.

> *Input arguments*

* `point` (*type:* `list` or `numpy.ndarray`): 2D point


## contains_polygons
```python
WorkspaceConstraint.contains_polygons(polygons)
```
Return True if polygons in the `polygons` list are part of the workspace.

> *Input arguments*

* `polygons` (*type:* list of `shapely.Polygon`): List of polygons


## get_geometry
```python
WorkspaceConstraint.get_geometry()
```
Return the workspace geometry

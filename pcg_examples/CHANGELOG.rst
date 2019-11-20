^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcg_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2019-11-21)
------------------
* Add missing test depedencies
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add tests for model and world generation
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add model factory examples for hinged doors
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add door mesh
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Convert meshes from DAE to STL
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add unique name to models with pcg\_ prefix
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#45 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/45>`_ from boschresearch/feature/import_world_file_in_world_generator
  Feature/import world file in world generator
* Contributors: Musa Morena Marcusso Manhaes

0.1.1 (2019-11-19)
------------------
* Delete data from CHANGELOGs to fix catkin_prepare_release issue not getting tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix CHANGELOG for catkin bug while retrieving tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#43 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/43>`_ from boschresearch/feature/world_generator_as_cmake_function
  Feature/world generator as cmake function
* Add PCG generated assets
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Generate worlds and models in build time
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add pcg_gazebo depedency
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Refactor extension from .yaml to .yml
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#42 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/42>`_ from boschresearch/feature/cmake_functions_model_generation_and_linter
  Feature/cmake functions model generation and linter
* Fix gazebo_ros dependency declaration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#38 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/38>`_ from boschresearch/feature/parse_xacro_to_urdf
  Feature/parse xacro to urdf
* Add generation of filled crate model group
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add crate mesh for example
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#34 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/34>`_ from boschresearch/feature/generate_mesh_from_shapely_geometry
  Feature/generate mesh from shapely geometry
* Refactor the name of the visual mesh argument
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#31 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/31>`_ from boschresearch/bugfix/collision_meshes_within_meshes
  Bugfix/collision meshes within meshes
* Install scripts
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#28 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/28>`_ from boschresearch/feature/dof_from_values_list
  Feature/dof from values list
* Set the example to use the choice policy for the sphere spawning
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#27 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/27>`_ from boschresearch/feature/world_generation_examples
  Feature/world generation examples
* Increase number of objects randomly generated
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add example of randomization in workspaces
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove enable contact and friction flags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add README and gitignore in folders where worlds and models are stored
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add launch file to start example worlds
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add YAML Jinja templates for physics configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add script to generate models from the local Jinja templates
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add script to generate all examples of models in the package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add world generator configuration for world with bouncy balls
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add world generator templates with ODE and Bullet
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add model template for bouncy ball
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add default configuration for physics engines
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add gazebo_ros dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Move model factory configuration files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#23 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/23>`_ from boschresearch/release/0.1.0
  Release/0.1.0
* 0.1.0
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix the initial version
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#20 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/20>`_ from boschresearch/feature/creating_dynamic_model_groups
  Feature/creating dynamic model groups
* 0.1.0
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add CHANGELOG
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#7 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/7>`_ from boschresearch/feature/travis_ci_integration
  Feature/travis ci integration
* Add missing dependencies
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Clean up CMakeLists.txt and install folders
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Initial commit
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

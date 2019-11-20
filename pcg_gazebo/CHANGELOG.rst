^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcg_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#45 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/45>`_ from boschresearch/feature/import_world_file_in_world_generator
  Feature/import world file in world generator
* Contributors: Musa Morena Marcusso Manhaes

0.1.1 (2019-11-19)
------------------
* Delete data from CHANGELOGs to fix catkin_prepare_release issue not getting tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix CHANGELOG for catkin bug while retrieving tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove warning on file not existing
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#43 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/43>`_ from boschresearch/feature/world_generator_as_cmake_function
  Feature/world generator as cmake function
* Add overwrite flag to avoid regeneration of files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add CMake function to generate world from PCG YAML configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#42 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/42>`_ from boschresearch/feature/cmake_functions_model_generation_and_linter
  Feature/cmake functions model generation and linter
* Process Jinja files in build time instead of configure time
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add option to merge nested models after processing Jinja file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use pcg\_ prefix for cmake macros
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Improve Jinja template processor
  * Add option to print output file instead of file export
  * Convert input parameters from the command line to correct type before template processing
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Install CMake functions with package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add includes for all CMake functions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add CMake functions for SDF and URDF file checks
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Include CMake functions to parse Jinja files
  * Process Jinja templates with parameter substitution
  * Generation of SDF file
  * Generation of static Gazebo model folder
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#40 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/40>`_ from boschresearch/feature/sdf_urdf_lint
  Feature/sdf urdf lint
* Set ground plane models from user input list
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#39 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/39>`_ from boschresearch/feature/sdf_urdf_lint
  Feature/sdf urdf lint
* Add option to generate static Gazebo model from URDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add lint scripts for SDF and URDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#29 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/29>`_ from boschresearch/feature/unpause_timeout
  Feature/unpause timeout
* Add unpause timeout to be used after robot is spawned
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#26 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/26>`_ from boschresearch/feature/add_collision_properties_to_model_factory_functions
  Feature/add collision properties to model factory functions
* Fix default name of the exported world file
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix conflicts with master
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#25 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/25>`_ from boschresearch/bugfix/expand_nested_models
  Bugfix/expand nested models
* Initialize node only when ROS topics or parameters are needed
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Use the PCG YAML loaders
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#23 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/23>`_ from boschresearch/release/0.1.0
  Release/0.1.0
* 0.1.0
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix the initial version
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#22 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/22>`_ from boschresearch/bugfix/transfer_jinja_parser_to_utils
  Bugfix/transfer jinja parser to utils
* Move jinja template parser to utils module
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#21 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/21>`_ from boschresearch/bugfix/model_group_from_sdf
  Bugfix/model group from sdf
* Add paused and gui flags
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add paused and gui flags to triggered Gazebo start script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add physics engine input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#20 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/20>`_ from boschresearch/feature/creating_dynamic_model_groups
  Feature/creating dynamic model groups
* 0.1.0
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add CHANGELOG
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use world generator from_dict and from_yaml loaders
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use spawn model service and unpause simulation after generated SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#18 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/18>`_ from boschresearch/feature/process_jinja_templates
  Feature/process jinja templates
* Move world Jinja template
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Exit script if no input world file is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add script to process a Jinja template
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Refactor name of the generation script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Rename script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#17 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/17>`_ from boschresearch/feature/jinja_template_parser
  Feature/jinja template parser
* Set roslaunch as test dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove simulation builder script from installation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add roslaunch unit tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add default parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove old script for builder server
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix script description
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Increase timeout to wait for message
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add step for spawning the model after generation of robot description
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#15 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/15>`_ from boschresearch/hotfix/process_urdf_before_spawn
  Hotfix/process urdf before spawn
* Use ROS logging
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Wait for spawn service after processing the robot description
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#12 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/12>`_ from boschresearch/hotfix/better_simulation_module_construction
  Hotfix/better simulation module construction
* Remove redundant exit
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add license header
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#7 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/7>`_ from boschresearch/feature/travis_ci_integration
  Feature/travis ci integration
* Remove old log
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add missing dependencies
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Install Python scripts
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#6 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/6>`_ from boschresearch/hotfix/remove_python3_shebang
  Remove python3 shebang from script
* Remove python3 shebang from script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#2 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/2>`_ from boschresearch/hotfix/unit_tests
  Hotfix/unit tests
* Fix opening the file to decode a template input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Initial commit
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcg_libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-12-05)
------------------
* Set pycollada v0.6 as requirement to solve parsing bug
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Reinstate is_hex test for input string conversion
* Set log level to ERROR
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix the creation of directory during build
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Disable hex test for strings
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add method to return the convex hull of the model's footprint
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#46 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/46>`_ from boschresearch/feature/hinged_door_creator
  Feature/hinged door creator
* Contributors: Musa Morena Marcusso Manhaes

0.1.2 (2019-11-21)
------------------
* Allow tests with DAE files only with Python 3.x
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Enabling again tests that required libspatialindex-dev
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add call for hinged door creator from dict
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add method to create door from mesh
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add functions to compute plane fit and major axis
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add input to set use_parent_model_frame for new joint
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add use_parent_model_frame flag and parse SDF elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add method to set use_parent_model_frame flag
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add hinged door abstraction from SimulationModel
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix access to link bounds
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add methods to remove links and joints by name
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add pose input for joint constructor
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#45 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/45>`_ from boschresearch/feature/import_world_file_in_world_generator
  Feature/import world file in world generator
* Fix SDF parsing test for elements with restricted number of string inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.1.1 (2019-11-19)
------------------
* Delete data from CHANGELOGs to fix catkin_prepare_release issue not getting tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix CHANGELOG for catkin bug while retrieving tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add method to initialize the world generator with world file
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* If SDF element is provided, test if world is nested within XML structure
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Improve assertion error messages with XML element name
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add error message for use of non-implemented method get_formatted_value_as_str in base class
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add new SDF parser elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Read fixed-model list from the world directly when running engines
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix typo in docstring
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add abstract attributes property that can be overwritten by derived classes
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Adding gui and fullscreen abstractions for SDF parser
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#44 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/44>`_ from boschresearch/bugfix/mesh_scaling
  Apply mesh scaling factor, when one is provided
* Apply mesh scaling factor, when one is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#43 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/43>`_ from boschresearch/feature/world_generator_as_cmake_function
  Feature/world generator as cmake function
* Add robot namespace to pose_gt declaration only when not equal none
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove debugging print
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#42 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/42>`_ from boschresearch/feature/cmake_functions_model_generation_and_linter
  Feature/cmake functions model generation and linter
* Disable mesh property test until  Travis build problem with libgeo is solved
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use SimulationModel merging function to expand nested models
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add XML element name to assertion error message
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add model merge function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add recursive refactoring of parameter method
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use pcg\_ prefix for cmake macros
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Replace namespace for robot_namespace to avoid collision with Jinja types
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Improve detection of integers in string for Python 2.x and 3.x
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix model version options
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set input argument log level to file output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#41 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/41>`_ from boschresearch/feature/add_sdf_scene
  Add SDF scene elements to parser
* Add SDF scene elements to parser
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#40 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/40>`_ from boschresearch/feature/sdf_urdf_lint
  Feature/sdf urdf lint
* Add method to set model as ground plane
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Allow retrieving multiple meshes from same link
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Test if physics engine block exists
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Allow multiple meshes imported from one file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set a warning in case the XML element is not valid
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix computation of footprint through triangulation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Test for existent footprints before generating occupancy grid
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Resolve URI only when requested
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set default logging level to ERROR
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#39 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/39>`_ from boschresearch/feature/sdf_urdf_lint
  Feature/sdf urdf lint
* Improve error message
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#38 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/38>`_ from boschresearch/feature/parse_xacro_to_urdf
  Feature/parse xacro to urdf
* Remove whitespace
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Test if workspace exists
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge links with fixed joints for urdf2sdf conversion
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add method to find values using input pattern
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix computation of combined pose
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix computation of final pose for model group elements
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add option to copy resources when generating static Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Return moments of inertia in matrix form
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove generation of package path for SDF conversion
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix computation of different between two poses
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove debugging messages
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add XACRO file parsing function and merging of massless links
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix computation of adjacent frames
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Check if Gazebo model directory path was returned
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Return the Gazebo model directory
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix SDF parsing functions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Return model directory is successful, None otherwise
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix parsing collision element name
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix parsing visual element name
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix parsing pose input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add pose and function to parse from SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add function to parse from SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix access to name and filename
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add SDF parser
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add sensor abstraction in simulation submodule
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add material as child element
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add missing scale attribute
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove whitespace
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add function to remove links and joints and test for massless links
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#36 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/36>`_ from boschresearch/bugfix/fix-conversion-of-joint-and-link-poses-when-converting-urdf-to-sdf
  Bugfix/fix conversion of joint and link poses when converting urdf to sdf
* Test creation of models from URDF structures with the correct poses
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set Pose.from_sdf as a static method
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Build graph from robot kinematic chain to find paths between links and compute absolute poses
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#34 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/34>`_ from boschresearch/feature/generate_mesh_from_shapely_geometry
  Feature/generate mesh from shapely geometry
* Test extruded model to static Gazebo model conversion
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Create meshes or copy resources when creating static Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set pose of  link created from mesh
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add room model creator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add mesh creator module
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Resolve Gazebo model and ROS package names
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Move log folder to home directory
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Delete generated meshes after tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test for extruded mesh creator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Refactor name of visual mesh input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add default output for PCG generated resources
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add creator for extruded polygons and allow mesh constructor to use trimesh input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use trimesh.Trimesh input to initiliaze Mesh object and export trimesh object if file does not exist
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Allow mesh input to be either a filename or a trimesh.Trimesh object
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#33 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/33>`_ from boschresearch/feature/export_to_gazebo_model
  Feature/export to gazebo model
* Test export_to_gazebo_model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix indentation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add method to export model group as a static Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add method to export model as a static Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix indentation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix setting version value if input is float
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix verification of version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add requirements for mesh intersection checks
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#32 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/32>`_ from boschresearch/bugfix/delete_lock_file_after_tasks_finish
  Delete the port lock file after the tasks finish
* Delete the port lock file after the tasks finish
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#31 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/31>`_ from boschresearch/bugfix/collision_meshes_within_meshes
  Bugfix/collision meshes within meshes
* Fix closing tag
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add libfcl-dev (dependency from python-fcl) as a dependency for pcg_libraries
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Disabling collision manager test for now until libspatialindex-dev is whitelisted in Travis CI
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add python depedency to pcg_libraries to obtain Python.h headers necessary for trimesh
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Install Python 2.7 dependencies for melodic Travis build
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove duplicated library
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add Python flexible collision library
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add new unit tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add unit test for collision checker
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Flag collision as true if any meshes and contained within other meshes
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#29 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/29>`_ from boschresearch/feature/unpause_timeout
  Feature/unpause timeout
* Fix lower limit
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Pause the simulation when the timeout is reached
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#28 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/28>`_ from boschresearch/feature/dof_from_values_list
  Feature/dof from values list
* Add option to set the DoF from a list of values
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#27 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/27>`_ from boschresearch/feature/world_generation_examples
  Feature/world generation examples
* Use assets manager instance as input to new engine
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Use volume instead of footprint are to pick models by size
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add test for mesh vertices contained in 2D workspace
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Replace input argument for get_model handle for assets manager instance
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Get handle for the assets manager instead of get_model
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Convert parsed parameters to float
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add assertion test error message
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix creation of ode block for contacts
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add poissons_ratio and elastic_modulus to Jinja macro
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Create collision properties dynamics in the SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add new SDF elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add Poisson's ratio and elastic modulus
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove enable flags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add SDF tags to contact block
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Assert the SDF object is not None
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add Jinja template for model.config file
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Return the raw parsed template if it is not an XML
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Check if lambda returns a scalar
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add test for hex inputs as string
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add new SDF elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add information on erroneous input in assertion check
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Test if element is None
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add new contact flag elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add new SDF elements
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#26 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/26>`_ from boschresearch/feature/add_collision_properties_to_model_factory_functions
  Feature/add collision properties to model factory functions
* Enable use of bounce element
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Set default parameters to None to signal that the default should be used
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove redundant creation of collision entity
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Allow lambda functions to be parsed for box, cylinder, mesh and sphere
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Remove contact element from collision
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Test sphere and cylinder model creators
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add input bounce parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add visual and collision property inputs for link creators
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add visual and collision properties inputs for mesh link creator
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add collision and visual properties inputs for cylinder, sphere and mesh creators
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add default logger
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add bounce macro
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Solve merge conflict with master
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix conflicts with master
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#25 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/25>`_ from boschresearch/bugfix/expand_nested_models
  Bugfix/expand nested models
* Expand nested models to convert SDF to URDF
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Use unique rot vector for orientation input in Pose
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Initialize rotation as rpy or quat depending on length of input vector
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add test of collision parameters to box factory function
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add bullet friction and logger
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add collision parameters as input to add_cuboid_link
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add message to assertion test
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add message to assertion test
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Parse path into ROS package URI
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add collision and visual parameters to box factory function
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Fix parsing of models and lights from SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Merge pull request `#24 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/24>`_ from boschresearch/feature/retrieve_models_from_usr_share
  Feature/retrieve models from usr share
* Test if resources are found in /usr/share folder
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Use string format for floating point
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Look into /use/share/gazebo-X/models folder for models
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Format integer and floats into strings
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Replace print by the PCG logger
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
* Fix access to child element <type>
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set default log level to ERROR
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#21 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/21>`_ from boschresearch/bugfix/model_group_from_sdf
  Bugfix/model group from sdf
* Test if list of model and lights is available before parsing the SDF elements
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#20 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/20>`_ from boschresearch/feature/creating_dynamic_model_groups
  Feature/creating dynamic model groups
* 0.1.0
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add CHANGELOG
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add docstrings
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add world generator unit tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use the from_dict constructor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove print of output XML element
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add from_dict constructor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add assertion tests for cuboid link inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use asset and engine manager for the generation of a new world configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set from_dict as static
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Return model directly from asset manager
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Generate random engine name if tag is missing
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Allow use of model groups to retrieve bounds
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add function to add a full model group
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix count of models locally and in subgroups
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add type input for parsing dict description assets
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove initial requirement for Gazebo models list to be empty
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add new unit test scripts
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Start adapting of world generator to use model groups generator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test YAML files for testing the YAML loader
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test box Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add unit tests for new package modules
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use random generation of strings
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test for nested model groups and import from SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Rename SDF unit test
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use assets and engines collections
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add collection manager classes to module
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add engine and constraint collections manager
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add object attributes for light configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Allow exceptions to interrupt parsing execution
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add assets manager for light, model, model factory and model groups
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add missing SDF elements to package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Update Gazebo models' list
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix setting the base class' value attribute
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Overload equality operator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add base class for managing collections of entities
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add constructor input for local collision checker instance
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Adapt world to use model groups per default
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add copy constructor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add import from dict function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add custom YAML loaders to parse ROS paths
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set default log level to warning
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add lights to model group and import from SDF function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set spot settings as optional
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Overload __eq_\_ operator for Pose object
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix testing if the XML element is available already
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use trimesh Scene instead of boolean operation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set value options as an attribute
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add elements to the <ode> block
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test for model groups
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add utils module
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add test for model groups
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Get copy from meshes bounds object
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Rename test file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add ModelGroup to subpackage
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add model group generator class
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Move model group to simulation subpackage
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Create ModelGroup class to manage sets of models
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#19 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/19>`_ from boschresearch/feature/gazebo_ros_path_resolve
  Feature/gazebo ros path resolve
* Fix the initialization for Mesh when input filename is None
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix unit test after finding the ROS package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Resolve mesh paths in the Mesh class
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use path class to resolve paths
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix testing of string types inputs for Python 2 and 3
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Import simulation packages locally
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add path class to pcg_gazebo module
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add ROS package name to Gazebo model information
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add unit test for path class
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove ROS tests, use nosetests only
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix line breaks
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove fix normals and fill mesh holes after loading
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add SDF/URDF path resolver class
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#18 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/18>`_ from boschresearch/feature/process_jinja_templates
  Feature/process jinja templates
* Parse physics engine inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Provide input arguments for physics engine constructors
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add assertion error messages
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix default argument of viscous_friction
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add encoding when parsing XML file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add Jinja template for basic models
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#17 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/17>`_ from boschresearch/feature/jinja_template_parser
  Feature/jinja template parser
* Add current input value when AssertionError is thrown
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix type for cone_model in the friction model option
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use Jinja renderer in unit test
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix type on use_dynamic_moi_rescaling
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add world file template
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add Jinja template renderer
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add world file template
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add macro to generate <physics> block
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add input for robotNamespace in pose_gt macro
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove generated test SDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use find_ros_package to resolve import paths
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#16 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/16>`_ from boschresearch/hotfix/bullet_upper_limit_for_friction
  Hotfix/bullet upper limit for friction
* Add random string to log file path to avoid conflicts on two pcg instances
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove upper limit for Bullet friction parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#15 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/15>`_ from boschresearch/hotfix/process_urdf_before_spawn
  Hotfix/process urdf before spawn
* Add gazebo_ros_control macro and input for ROS version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#12 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/12>`_ from boschresearch/hotfix/better_simulation_module_construction
  Hotfix/better simulation module construction
* Add trimesh's optional dependency scipy
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove a slash when refactoring $(find pkg)
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add joint log messages to PCG log output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add networkx as package dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Set IMU parameters for older SDF versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add networkx (a trimesh dependency)
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add more constructor inputs to configure the joint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add plugins to models
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add parser for $(find pkg) format
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix setting internal attributes from constructor's inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add initialization of kinect ROS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Test if parent is world before searching in model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add more configuration inputs in constructor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove redundant distortion input and add kinect plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix type of ray sensor SDF output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add method to set the internal sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#9 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/9>`_ from boschresearch/hotfix/mesh_box_approximation
  Hotfix/mesh box approximation
* Use the bounds of the mesh to compute the approximated box
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix the computation of approximated box models from mesh
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#8 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/8>`_ from boschresearch/hotfix/broken-random-engine-assets-list
  Hotfix/broken random engine assets list
* Renaming Link module and transformation fixes
  * Rename SimulationObject to Link
  * For Python 2.x, test input name for unicode and str types
  * Fix composed pose transformation for retrieving footprints
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use cached footprint polygons for repeated workspace tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix access to constraint label
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Generate z_levels from z_limits if None is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Replace SimulationObject for Link
  SimulationObject was refactored to respect the
  nomenclature used in Gazebo, SDF and URDF robot
  descriptions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Rename simulation.SimulationObject to simulation.Link
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#7 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/7>`_ from boschresearch/feature/travis_ci_integration
  Feature/travis ci integration
* Set kinetic to allow failures at Travis CI
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Use openscad to test boolean operations
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add blender as trimesh dependency for boolean operations
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix invalid mismatched tag
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add libxml2-utils as depedency for xmllint
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Replace trimesh[all] for trimesh for missing glooey
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update requirements list
  * Set the complete installation of trimesh
  * Add pycollada for parsing of DAE files by trimesh
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Clean up script
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add missing dependencies
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add python-pip as depedency
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#5 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/5>`_ from boschresearch/hotfix/remove_virtualenv_dependency
  Hotfix/remove virtualenv dependency
* Refactor comment
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove virtualenv as a dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove old URDF test file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#2 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/2>`_ from boschresearch/hotfix/unit_tests
  Hotfix/unit tests
* For Python 2.x, test string input for unicode type
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix test of input value to is_scalar
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Test for unicode input for Python 2.x
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Catch type error in value test methods
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Print exception message, not URDF content
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add option for single process model generation if n_processes=None
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use jinja2 instead of yasha to parse templates
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix access to XML element name
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix ROS test installation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove duplicate test
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Use rospkg to solve paths and fix the access to URDF element name
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove whitespaces
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Rename test URDF files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#1 <https://github.com/boschresearch/pcg_gazebo_pkgs/issues/1>`_ from boschresearch/feature/installation_instructions
  Feature/installation instructions
* Fix verification of scalar input in static method
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix print of pose vector
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix conversion of rpy2quat
  No longer using the pyquaternion structure
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Remove rospkg from requirements
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Install missing Python dependencies in the user space
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Initial commit
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

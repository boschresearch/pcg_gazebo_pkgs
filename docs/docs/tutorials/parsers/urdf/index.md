ROS applications make use of the [URDF format](https://wiki.ros.org/urdf/XML) for the robot description.
This package includes a parsers also for URDF to make it possible to improve
the cross-conversion to and from SDF regarding various simulation-specific tags that 
will mostly be parsed only for the `ODE` physics engine at the moment. 

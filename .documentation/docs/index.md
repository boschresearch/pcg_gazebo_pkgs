# Procedural Generation for Gazebo

[![Build Status](https://travis-ci.org/boschresearch/pcg_gazebo_pkgs.svg?branch=master)](https://travis-ci.org/boschresearch/pcg_gazebo_pkgs)
[![GitHub issues](https://img.shields.io/github/issues/boschresearch/pcg_gazebo_pkgs.svg)](https://github.com/boschresearch/pcg_gazebo_pkgs/issues)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](https://github.com/uuvsimulator/uuv_simulator/blob/master/LICENSE)

The Procedural Generation for Gazebo package is an Open Source
Project extending the simulation capabilities of the robotics simulator Gazebo
for automation and scripting of Gazebo simulations.

Visit the [documentation page](https://boschresearch.github.io/pcg_gazebo_pkgs/) for more information.

## Purpose of the project

This software is a research prototype.

The software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

## Requirements

### Installation from source

Clone the repository into your catkin workspace

```
cd ~/catkin_ws/src
git clone https://github.com/boschresearch/pcg_gazebo_pkgs.git
```

then install the dependencies (replace `kinetic` for the ROS distribution you are using)

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y -r
```

and then run `catkin build`.

#### Installation of Python dependencies

The Python dependencies for the `pcg_gazebo` library can be found in [`pcg_libraries/requirements.txt`](https://github.com/boschresearch/pcg_gazebo_pkgs/blob/master/pcg_libraries/requirements.txt).
Since some of them cannot be resolved using `rosdep`, the missing dependencies are going
to be installed in `pip` using the default `python` version in the target system in the `user` 
space.

## License

Procedural Generation for Gazebo is open-sourced under the Apache-2.0 license. See the [LICENSE](https://github.com/boschresearch/pcg_gazebo_pkgs/blob/master/LICENSE) file for details.

For a list of other open source components included in Procedural Generation for Gazebo package, see the file [3rd-party-licenses.txt](https://github.com/boschresearch/pcg_gazebo_pkgs/blob/master/3rd-party-licenses.txt).

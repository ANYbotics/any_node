# ANY Node

## Overview

Set of wrapper packages to handle multi-threaded ROS nodes.

The packages contained have been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

Contact: leggedrobotics@ethz.ch

## Building

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/any_node/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/any_node/job/master/)

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/any_node/issues).

## Packages

This is only an overview. For more detailed documentation, please check the packages individually.


### any_node

ROS node wrapper with some convenience functions using *any_worker*.

### any_worker

High resolution version of the ROS worker.

### param_io

Wrapper for the ROS param get and set functions outputting warnings if a parameter has not been found.

### signal_handler

Contains a static signal handling helper class.




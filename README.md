# ROS Gazebo Gym Examples

[![ROS Gazebo Gym Examples](https://github.com/rickstaa/ros-gazebo-gym-examples/actions/workflows/ros-gazebo-gym-examples.yml/badge.svg)](https://github.com/rickstaa/ros-gazebo-gym-examples/actions/workflows/ros-gazebo-gym-examples.yml)
[![ROS Test](https://github.com/rickstaa/ros-gazebo-gym-examples/actions/workflows/ros_test.yml/badge.svg?branch=noetic)](https://github.com/rickstaa/ros-gazebo-gym-examples/actions/workflows/ros_test.yml)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-examples)](https://github.com/rickstaa/ros-gazebo-gym-examples/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![DOI](https://zenodo.org/badge/453634930.svg)](https://zenodo.org/badge/latestdoi/453634930)

Welcome to the ROS Gazebo Gym Examples repository! In this repository, you will find a diverse range of examples demonstrating how to utilize the environments provided by the [ros-gazebo-gym](https://rickstaa.dev/ros-gazebo-gym) package effectively.

## Examples

The following examples are currently available:

*   [start\_panda\_training.launch](./launch/start_panda_training.launch): Launches a SAC agent for training in the Panda robotics environment.

For comprehensive guidance on utilizing these examples and leveraging the capabilities of the [ROS Gazebo Gym](https://rickstaa.dev/ros-gazebo-gym) package, please refer to the [documentation](https://rickstaa.dev/ros-gazebo-gym/get_started/usage.html#usage-examples).

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [contributing guidelines](CONTRIBUTING.md) in the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) package before contributing to this repository.

> \[!NOTE]\
> We used [husky](https://github.com/typicode/husky) instead of [pre-commit](https://pre-commit.com/), which is more commonly used with Python projects. This was done because only some tools we wanted to use were possible to integrate the Please feel free to open a [PR](https://github.com/rickstaa/ros-gazebo-gym-examples/pulls) if you want to switch to pre-commit if this is no longer the case.

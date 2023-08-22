# Ros-gazebo-gym-examples

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-examples)](https://github.com/rickstaa/ros-gazebo-gym-examples/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](contributing.md)

This repository contains several usage examples for the gymnasium environments in the [ros-gazebo-gym](https://rickstaa.github.io/ros-gazebo-gym) package.

## How to use

Below you can find instructions on how to use the examples in this repository. For more comprehensive instructions on how to use this package or the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework, please refer to the [ros\_gazebo\_gym](https://rickstaa.dev/ros-gazebo-gym) documentation.

### Build the catkin workspace

As the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) and [ros-gazebo-gym-examples](https://github.com/rickstaa/ros-gazebo-gym-examples) packages are currently not available on the ROS package index, you will need to construct the catkin workspace manually. Comprehensive instructions for this process can be found in the [ROS documentation](https://wiki.ros.org/catkin/Tutorials/create_a_workspace). To simplify this, an easily accessible catkin workspace repository containing all necessary packages has been provided [here](https://github.com/rickstaa/ros-gazebo-gym-ws).

> \[!NOTE]\
> Please note that you must ensure all system dependencies are installed before building the workspace. This can be achieved by using the following [rosdep](http://wiki.ros.org/rosdep) command:
>
> ```bash
> rosdep install --from-path src --ignore-src -r -y
> ```

### How to run the package

After the workspace has successfully been built and sourced, you can run any of the example projects by launching the `start_training` launch file in the project folder using the `roslaunch` command:

```bash
roslaunch ros_gazebo_gym_examples start_training.launch
```

This will launch the `start_training` launch file, which will download all the required dependencies for a given example and run the gazebo simulator. After this, the agent will start training in the environment. Each example project in the  uses [ros\_gazebo\_gym\_examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package uses the Soft-Actor Critic algorithm of the [stablebaselines3](https://stable-baselines3.readthedocs.io/en/master/) package.

> \[!WARNING]
> If you're attempting to run the package on Ubuntu 20.04, you might encounter issues stemming from conflicting versions of the gymnasium and Numpy packages (see [this issue](https://github.com/ros/rosdistro/pull/38242)). You can address this by installing specific versions of gymnasium and Numpy using pip3:
>
> ```bash
> pip install -r requirements/requirements.txt
> ```

### Using virtual environments

If you want to try out the examples inside a virtual environment, you are you are recommended you to use a native Python virtual environment (using the [venv](https://docs.python.org/3/library/venv.html) package) since [Anaconda](https://www.anaconda.com/) is not yet fully compatible with ROS (see [this issue](https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/)).

#### Setup virtual environment

First, make sure that [ROS noetic](https://wiki.ros.org/noetic) has been installed on your system (See the [ROS site](https://wiki.ros.org/noetic) for instructions). Following, install the `python3-venv` package and create a virtual python3 environment using the following command:

```bash
python3 -m venv ./venvs/ore --system-site-packages
```

After this environment is created, you can activate it using:

```bash
source ./venv/ore/bin/activate
```

> \[!IMPORTANT]\
> Please note that the `--system-site-packages` flag in the command above is required for the ROS system packages to be available inside the virtual environment.

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [contributing guidelines](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/contributing.md) in the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) package before contributing to this repository.

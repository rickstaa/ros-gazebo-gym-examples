# Ros-gazebo-gym usage example projects

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-examples)](https://github.com/rickstaa/ros-gazebo-gym-examples/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-green)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-orange.svg)](contributing.md)

This repository contains several usage examples for the gym environments in the [ros-gazebo-gym](https://rickstaa.github.io/ros-gazebo-gym) package.

## How to use

### Build the catkin workspace

You have to manually build the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) and [ros-gazebo-gym-examples](https://github.com/rickstaa/ros-gazebo-gym-examples) packages since they are not yet been released to the ROS package index. A catkin workspace repository that contains these packages can be found [here](https://github.com/rickstaa/ros-gazebo-gym-ws). For more information on building catkin workspaces; see the [ROS documentation](https://wiki.ros.org/catkin/Tutorials/create_a_workspace). Please note that you have to ensure that all the system dependencies are installed before building the workspace. This can be achieved by using the following [rosdep](http://wiki.ros.org/rosdep) command:

```bash
rosdep install --from-path src --ignore-src -r -y
```

### How to run the package

After the workspace has successfully been built and sourced, you can run any of the example projects by launching the `start_training` launch file in the project folder:

```bash
roslaunch ros_gazebo_gym_panda_example start_training.launch
```

The `ros-gazebo-gym` package will download all the required dependencies and run the gazebo simulator. After this, the agent will start training in the environment. Each example project contains a [stablebaselines3](https://stable-baselines3.readthedocs.io/en/master/) RL algorithm example.

### Using virtual environments

If you want to try out the examples inside a virtual environment you are you are recommended you to use a native python virtual environment (using the [venv](https://docs.python.org/3/library/venv.html) package) since [Anaconda](https://www.anaconda.com/) is not yet fully compatible with ROS (see [this issue](https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/)).

#### Setup virtual environment

First, make sure that ROS noetic has been installed on your system (See the [ROS site](https://wiki.ros.org/noetic) for instructions). Following, install the `python3-venv` package and create a virtual python3 environment using the following command:

```bash
python3 -m venv ./venvs/ore --system-site-packages
```

Please note that the `--system-site-packages` flag required you to reach the ROS python packages system. After this environment is created, you can activate it using:

```bash
source ./venv/ore/bin/activate
```

Finlay installs the python requirements of each example using the following command:

```bash
pip install -r <EXAMPLE_FOLDER>/requirements.txt
```

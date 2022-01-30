# Ros-gazebo-gym usage example projects

This repository contains several usage examples for the gym environments in the [ros-gazebo-gym](rickstaa.github.io/ros-gazebo-gym) package.

## How to use

### Install the python dependencies

When trying out the examples, you are recommended to install the dependencies in a virtual environment. We recommend you to use a native python virtual environment (using the [venv](https://docs.python.org/3/library/venv.html) package) since [Anaconda](https://www.anaconda.com/) is not yet fully compatible with ROS (see [this issue](https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/)).

#### Use virtual environment

When you want to use the native virtual environments, you should install ROS noetic directly on your system (See the [ROS site](https://wiki.ros.org/noetic) for instructions). After you installed ROS, you can install the `python3-venv` package and create a virtual python3 environment using the following command:

```bash
python3 -m venv ./venvs/ore --system-site-packages
```

Please note that the `--system-site-packages` flag required you to reach the ROS python packages system. After this environment is created, you can activate it using:

```bash
source ./venv/ore/bin/activate
```

You can then install the python requirements of each example using the following command:

```bash
pip install -r <EXAMPLE_FOLDER>/requirements.txt
```

### How to run the package

After you set up your virtual environment, you have to create a catkin workspace that contains both the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) and [ros-gazebo-gym-examples](https://github.com/rickstaa/ros-gazebo-gym-examples) repositories. A workspace repository that contains these packages can be found [here](https://github.com/rickstaa/ros-gazebo-gym-ws). See the [ROS documentation](http://wiki.ros.org/noetic) for more information on creating and building catkin workspaces; see the [ROS documentation](https://wiki.ros.org/catkin/Tutorials/create_a_workspace). After the workspace has successfully been built and sourced, you can run any of the example projects by launching the start\_training launch file in the project folder:

```bash
roslaunch ros_gazebo_gym_panda_example start_training.launch
```

The ros-gazebo-gym package will download all the required dependencies and run the gazebo simulator. After this, the agent will start training in the environment. Each example project contains a [stablebaselines3](https://stable-baselines3.readthedocs.io/en/master/) RL algorithm example.

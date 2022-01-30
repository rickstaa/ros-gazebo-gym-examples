#!/usr/bin/env python3
"""A small example that can be used to train the DQN agent of the Openai
stable-baselines3 package on the Panda environment of the ros_gazebo_gym package.
"""
import functools
from pathlib import Path

import gym
import numpy
import rospkg
import rospy
import torch
import ros_gazebo_gym  # noqa: F401
from stable_baselines3 import SAC

if __name__ == "__main__":
    rospy.init_node(
        "ros_gazebo_gym_panda_training_example", anonymous=True, log_level=rospy.WARN
    )

    # Retrieve input arguments
    try:
        control_type = rospy.get_param("~control_type")
    except KeyError:
        control_type = "effort"

    # Init ros_gazebo_gym Panda environment
    rospy.loginfo("Creating ros_gazebo_gym Panda gym environmen...")
    task_and_robot_environment_name = rospy.get_param(
        "/ros_gazebo_gym_panda_example_params/task_env"
    )
    env = gym.make("PandaReach-v0", control_type=control_type)
    rospy.loginfo("Panda gym environment created.")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("ros_gazebo_gym_panda_example")
    outdir = pkg_path + "/training_results"
    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    alpha = rospy.get_param("/ros_gazebo_gym_panda_example_params/alpha")
    gamma = rospy.get_param("/ros_gazebo_gym_panda_example_params/gamma")
    n_episodes = rospy.get_param("/ros_gazebo_gym_panda_example_params/n_episodes")
    n_steps = rospy.get_param("/ros_gazebo_gym_panda_example_params/n_steps")
    inference = rospy.get_param("/ros_gazebo_gym_panda_example_params/inference")
    inference_n_steps = rospy.get_param(
        "/ros_gazebo_gym_panda_example_params/inference_n_steps"
    )
    inference_n_episodes = rospy.get_param(
        "/ros_gazebo_gym_panda_example_params/inference_n_episodes"
    )
    total_timesteps = n_steps * n_episodes

    # Set max_episode_steps
    env._max_episode_steps = n_steps

    # Convert goal gym env to normal gym env
    env = gym.wrappers.FlattenObservation(env)

    # Initializes the algorithm that we are going to use for learning
    torch.cuda.empty_cache()
    model = SAC("MlpPolicy", env, verbose=1, gamma=gamma, learning_rate=alpha)

    # Train the algorithm
    rospy.loginfo("Starting Learning...")
    model.learn(total_timesteps=total_timesteps, log_interval=4)
    rospy.loginfo("Training finished")

    # Safe model
    model_save_dir = Path(outdir).joinpath("panda_baselines")
    model.save(str(model_save_dir))

    # Run inference
    if inference:
        rospy.logwarn("Starting inference...")
        model = SAC.load(model_save_dir)
        obs = env.reset()
        highest_reward = 0
        for x in range(inference_n_episodes):
            cumulated_reward = 0
            done = False

            # Initialize the environment and get first state of the robot
            observation = env.reset()
            state = "".join(map(str, observation))

            # Take episode
            for i in range(inference_n_steps):
                rospy.logwarn("############### Take Step=>" + str(i))

                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)
                cumulated_reward += reward
                if highest_reward < cumulated_reward:
                    highest_reward = cumulated_reward
                env.render()
                if done:
                    rospy.logdebug("DONE")
                    obs = env.reset()
                    last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
        rospy.logwarn("Inference finished.")

        # Display inference score
        last = last_time_steps.tolist()
        last.sort()
        rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
        rospy.loginfo(
            "Best 100 score: {:0.2f}".format(
                functools.reduce(lambda x, y: x + y, last[-100:]) / len(last[-100:])
            )
        )
        env.close()

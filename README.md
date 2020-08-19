# TOWR_pybullet
- a simulation test bed for TOWR trajectories in Pybullet

## Introduction:
The software development of the robotics community is greatly biased towards the usage of C++ and this is greatly justified by the speed that this language provides us with. However the growing field of *Robot Learning* requires faster prototyping and flexible simulation arrangments to incorprate various learning based frameworks.The arrival of modules like *Tensorflow,Pytorch,Stable Baselines* for learning and *Pybullet* for simulation has made *Python*, a very attractive choice for trainig and testing learning based agents in robots.This codebase is and attempt to build a pythonic wrapper arround *[TOWR - Trajectory Optimizer for Walking Robots](https://github.com/ethz-adrl/towr)*.

## TOWR:
<p align="center">
   <img width="1000" height="500" src="https://github.com/lok-i/towr_pybullet/blob/master/media/towr.gif">
</p>

TOWR(Trajectory Optimizer for Walking Robots), is a state of the art light-weight and extensible C++ library for trajectory optimization for legged robots developed in ETH Zurich by Alexander Winkeler and his team.It proposes to address the problem of integrated motion planning as a non-linear optimization problem and there by provides a deasible kinematics solution.This framework is very versatile as address the robot's constraints (from its dynamic and kinematic model),the world constraints and the goal constraints.Having been tested across various terrain, robots,and gait types it also allows the users to import their own robot model or add custom constraints and terrain models.

## Our work:

TOWR is entirely developed in C++ and has inbuilt support to visualize the generated trajectories.However, there is no available code for using TOWR with a physics simulator.Their paper shows the validation of towr in Gazebo, which requires
the usage additional ROS packages.This is quite cumbersome and not very straightforward for the robot learning community givent the ease that *Pybullet* provides us with.In the repo, we have built python functions to visualize aswell as simulate the TOWR generated trajectories on the robot platform ANYmal with different gait types and multiple terrains(flat ground and stairs as of now)

## Current Features

<p align="center">
   <img width="500" height="400" src="https://github.com/lok-i/towr_pybullet/blob/master/media/stairs.gif">
</p>
<p align="center">
   <img width="300" height="250" src="https://github.com/lok-i/towr_pybullet/blob/master/media/turn.gif">
   <img width="300" height="250" src="https://github.com/lok-i/towr_pybullet/blob/master/media/trot.gif">
</p>



## Ongoing Work:

* Building the other terrain models like block, chimney, Gap, Slope, Chimney, ChimneyLR, as physical bodies with realistic material properties. However, yoo can still visualize the trajectory but not simulate it.

* Adding support for other robots and feature to import user defined robots.

* Implementation of a class of controllers, that could actively track these generated trajectories.

* Inbuilt support for learning based frameworks so as to obtain optimal control and planning through Reinforcement Learning,Immitation Learning, etc. 

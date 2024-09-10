# Control for Robotics: from Optimal Control to Reinforcement Learning 

This repository contains the implementation with the corresponding results of the assignments of the practical part of the Control for Robotics course at TUM. The theoretical part of the course covered optimal control, learning-based control, and reinforcement learning principles from the perspective of robotics applications.

## Assignment 1 - Optimal Control and Dynamic Programming

### Problem 1
In this task, a robot vacuum cleaner navigates a grid world to reach its charging station while removing dirt and avoiding obstacles. The goal is to find the optimal policy for moving through the grid with varying cell costs. The solution involves using value iteration to calculate the cost-to-go and the optimal control sequence for the robot.

### Problem 2
This problem involves stabilizing an inverted pendulum at the upright position using approximate dynamic programming. The task requires linearizing the nonlinear system, discretizing the dynamics, and implementing a discrete-time controller that minimizes a quadratic cost function to stabilize the pendulum.

## Assignment 2 - Model-Based Iterative Linear Quadratic Control

### Problem 1 
This problem involves designing an LQR and ILQC controller which drives the mobile robot shown below in a straight line  
![Screenshot from 2024-09-10 10-32-28](https://github.com/user-attachments/assets/c2458f74-54ba-4683-8528-72de3892b55b)

For a desired reference of y = -5 and zero heading, the evolution of the states is shown below
![states_0_pi](https://github.com/user-attachments/assets/fe2a84c1-c8a3-4000-a85a-f7824fb923d6)


### Problem 2
In this problem, the task was to design controllers that allow a quadrotor to reliably reach a predefined goal and/or pass through predefined via-points. The state
state x of this robot is defined by the position of its center of mass (px , py , pz ) and the body orientations
(ϕ, θ, ψ) (roll, pitch, yaw), and their derivatives.
Both LQR and ILQC controllers were implemented and their performances compared. 
For an initial state of [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and goal state of [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], the simulation results for the LQR and ILQC are shown below:

![plots_x_goal_given](https://github.com/user-attachments/assets/46193c7a-9b08-4bf2-ba23-a397ed6b8853)

![ilqc_controller](https://github.com/user-attachments/assets/e2cbcf7d-051d-4737-a3b9-a7a0e0a11c3d)

## Assignment 3 - Model Predictive Control and System Identification

### Problem 1  
In this assignment, the task was to implement model predictive control (MPC) to solve a standard benchmark
problem in control and reinforcement learning: driving an under-powered car to the top of a hill. The dynamics of the car was given as 
v_{k+1} = v_{k} + 0.001a_{k} - 0.0025 * cos(3p_{k})
p_{k+1} = p_{k} + v_{k+1}
with position p and velocity v
The implementation produced the following simulation outputs for a goal stat of [v, p] = [0, 0.5]:

![Trajectories3_1](https://github.com/user-attachments/assets/09e2541d-21e1-4eb4-9d79-b5a93a671ceb)

### Problem 2
Assuming that the dynamics of mountain car has changed to v_{k+1} = v_{k} + \alpha a_{k} - \beta * cos(3p_{k}), with unknown parameters \alpha and \beta, the task was to estimate the optimal parameters from an input output dataset using Bayesian Linear Regression. Results are shown below

![Parameter Estimation](https://github.com/user-attachments/assets/0388db69-5ffb-42e7-b8d0-5992d53d22b9)

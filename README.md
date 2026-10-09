# CARGO: Cooperative Aerial Robotics for Good Operations

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)

**CARGO** is a multi-UAV cooperative cargo transport framework developed to enable multiple Unmanned Aerial Vehicles (UAVs), specifically quadcopters, to work together to transport payloads. By distributing the payload across multiple UAVs, this system increases transport capacity and maintains flight stability. 

This project was developed as part of the EL4060 course at the School of Electrical Engineering and Informatics, Institut Teknologi Bandung (STEI ITB).

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture (CARGO V2)](#system-architecture-cargo-v2)
- [Formation Control Algorithm](#formation-control-algorithm)
- [Prerequisites](#prerequisites)
- [Demo (Gazebo)](#demo-gazebo)
- [References](#references)

## Project Overview
Transporting dynamic payloads using a single UAV comes with limitations in carrying capacity and stability. CARGO solves this by utilizing a multi-UAV swarming approach. The project is divided into three core focus areas:
1. **Formation Control**: Algorithms for maintaining rigid formations and swarming during maneuvers.
2. **State Estimation**: Predicting and estimating the payload's state while the quadcopters are in motion.
3. **Implementation & Code Design**: A robust, modular ROS 2-based framework for onboard computing on each quadcopter.

## System Architecture (CARGO V2)
The current framework (CARGO V2) utilizes a modular, layered architecture **based on Aerostack2** with minor modifications and adjustments to be able to work seamlessly with **ArduPilot**. It ensures that core system functionalities are separated into well-defined, independent modules.

- **Firmware-Agnostic**: The system introduces a compatibility layer that separates the high-level logic from middleware (like MAVROS), allowing for easy transitions between different autopilot firmwares.
- **Independent Agents**: Each drone operates as a standalone agent with its own *Behavior Execution Control*, *Basic Robot Functions* (Motion Controller & State Estimator), and *Sensor Actuator Interface*.
- **Communication Layer**: Agents communicate their internal states via a shared network layer, which also connects to the Ground Control Station (GCS) for centralized user monitoring and mission planning.

## Formation Control Algorithm
The framework implements a distributed formation control system modeled for single-integrator agents in a 2D space. The hybrid control strategy relies purely on local relative position information:
- **Gradient Controller (Artificial Potential Field)**: Uses attractive forces to guide agents to their target positions within the formation and repulsive forces to avoid collisions with other agents or obstacles.
- **Consensus Term**: Synchronizes velocity and heading across the swarm to ensure smooth and coordinated movements.
- Control gain matrices are computed by solving Semidefinite Programming (SDP) problems to guarantee system stability and allow coordinate transformations from local to global frames.

## Prerequisites
To run this project, you will need the following dependencies installed:
- **Ubuntu** (22.04 recommended)
- **ROS 2 Humble**
- **Gazebo Harmonic**
- **MAVROS**
- **ArduPilot SITL** (for simulation)
- **MATLAB** (optional, for algorithm pre-testing)

## Demo (Gazebo)
*The following videos demonstrate the multi-UAV cooperative transport simulation running in Gazebo under different control modes. Click the images to watch the demos on YouTube.*

### 1. Manual Control
[![Manual Control Demo](https://img.youtube.com/vi/aQZQLfCi9bs/hqdefault.jpg)](https://youtu.be/aQZQLfCi9bs)

### 2. Autonomous (2 Drones)
[![Autonomous 2 Drone Demo](https://img.youtube.com/vi/rVeePUR0jfQ/hqdefault.jpg)](https://youtu.be/rVeePUR0jfQ)

### 3. Autonomous (3 Drones)
[![Autonomous 3 Drone Demo](https://img.youtube.com/vi/IFAns-RVuko/hqdefault.jpg)](https://youtu.be/IFAns-RVuko)

## References
### Related Works
- Fernandez-Cortizas, M., et al. (2023). "Aerostack2: A software framework for developing multi-robot aerial systems."
- Fathian, K., et al. (2018). "Distributed formation control and navigation of fixed-wing UAVs at constant altitude."
- Ma'Arif, A., et al. (2021). "Artificial potential field algorithm for obstacle avoidance in UAV quadrotor for dynamic environment."

# MultiAgentUAV300

# Multi-UAV Autonomous Exploration Framework

This repository contains the custom ROS 2 packages, launch files, nodes, and configuration files developed for my dissertation/project on autonomous UAV exploration in a GPS-denied simulation environment.

The project focused on integrating PX4-based offboard flight control with ROS 2 mapping, planning, frontier-based exploration, and multi-UAV operation. The framework was developed and tested in simulation and was extended from a single-UAV baseline to a multi-UAV architecture, with additional experiments in map merging and scalability.

## What this repository contains

This repository mainly contains the code I developed or modified for the project, including:

- custom ROS 2 package(s) for UAV exploration and control integration
- launch files for single-UAV and multi-UAV bringup
- custom nodes for:
  - frontier selection
  - waypoint following
  - map-to-PX4 goal conversion
  - odometry conversion
  - scan frame rewriting
- navigation and planner configuration files
- multi-agent launch and namespacing setup

## Main developed package

The main custom package developed for this project is:

- `drone_slam`

This package contains the main autonomy pipeline used in the experiments.

## External software, frameworks, and dependencies

This project depends on a number of external packages and frameworks which are not fully re-uploaded in this repository. These are acknowledged here because they formed an important part of the overall system:

- **PX4 Autopilot** for low-level flight control and offboard mode
- **Micro XRCE-DDS Agent** for PX4-ROS 2 communication
- **ROS 2 Humble**
- **Gazebo / gz-sim Harmonic** for simulation
- **QGroundControl** for UAV monitoring and manual interaction
- **Nav2** for path planning
- **slam_toolbox** and/or **RTAB-Map** for mapping/localisation work carried out during development  
  - [edit this line to reflect exactly what was used in the final system]
- **m-explore / multirobot_map_merge** for map merging experiments
- **ros_gz_bridge** for ROS-Gazebo topic bridging
- **tf2_ros** for TF publishing and transform handling
- **px4_ros_com** for PX4 ROS 2 integration
- **[YOLO / YOLO-ROS package name here]** for object-detection related experiments, if applicable  
  - [replace with the exact package name if used]
- **[Supervisor-provided UAV model / simulation assets]**  
  - The UAV model used in simulation was provided separately and may not be fully redistributable in this repository.

## Important note on repository contents

Not all software used in the project is uploaded directly here. In particular, some external dependencies, simulator assets, and provided models are not included in full because they are:

- third-party packages maintained elsewhere
- large external frameworks better installed from their original sources
- supervisor-provided or externally supplied assets
- not originally authored by me

This repository should therefore be treated as the custom development layer built on top of those external tools.

## Files not fully included here

Some parts of the complete project environment may need to be obtained separately, including:

- PX4 Autopilot source and SITL setup
- gz-sim Harmonic simulation environment
- QGroundControl
- Micro XRCE-DDS Agent
- external mapping/planning packages
- map merging packages
- any supervisor-provided drone model or world/model assets not owned by me
- any large datasets, logs, or generated simulation outputs

## What was developed in this project

The main technical work in this project involved:

- integrating PX4 offboard control with ROS 2 autonomy
- converting PX4 odometry into ROS-compatible odometry and TF
- building a frontier-based exploration pipeline for UAVs
- using Nav2 to generate paths through known free space
- converting planned paths into waypoint goals suitable for PX4 execution
- restructuring the system from single-UAV to multi-UAV operation using namespacing and parameterised launch design
- experimenting with map merging and merged-map exploration
- demonstrating architectural scalability to additional UAVs

## Reproducibility

To run this project, the required external dependencies must first be installed and configured. This repository is intended to provide the custom code and configuration developed during the project, rather than a complete standalone installation of every dependency used.

## Acknowledgements

This project builds on several open-source robotics frameworks, particularly PX4, ROS 2, Nav2, Gazebo, and map merging / SLAM tools. It also used a UAV simulation model and related assets provided during the project.

## Notes

If you are reviewing this repository for academic purposes, the most important custom code is contained in the `drone_slam` package and the associated launch/configuration files.

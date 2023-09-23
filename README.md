# Robotics Project - 7825_Template.cpp

## Overview

This is a Robotics project that was created by Faran Bhatti on Sept 10th, 2022. It focuses on multiple labs, each tackling specific tasks in robot operations like coordinate transformations, camera calibration, forward and reverse kinematics, and more. The project integrates OpenCV functionalities for computer vision tasks and also involves augmentations using virtual and real cameras.

## Table of Contents

- [Description](#description)
- [Files](#files)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Author](#author)

## Description

This Robotics project simulates and visualizes robot tasks in different lab sessions. The labs cover:

1. Coordinate Transforms 2D
2. Coordinate Transforms 3D
3. Virtual Camera
4. Camera Calibration
5. Forward Kinematics
6. Inverse Kinematics
7. Trajectories
8. Object Tracking
9. Deep Neural Network (DNN)

## Files

1. `7825_Template.cpp` - Main project file.
2. `camera.h` - Header file for camera operations.
3. `robot.h` - Header file for robot operations.

## Dependencies

- OpenCV
- cvui (for GUI elements)
- Standard C++ libraries

## Installation

1. Install OpenCV and required dependencies.
2. Clone the project repository.
3. Compile the project using a compatible C++ compiler.

```bash
g++ -o robot 7825_Template.cpp `pkg-config --cflags --libs opencv`

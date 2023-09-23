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

## Files

- **.gitattributes & .gitignore**: Git configuration and ignore rules.
- **7825_Template**: Files related to the 7825 template.
  - `7825_Template.cpp`
  - `7825_Template.sln`
  - `7825_Template.vcxproj`
  - `7825_Template.vcxproj.user`
- **Camera**:
  - `Camera.cpp`: The "camera.h" file defines a CCamera class that manages both virtual and real camera operations. Utilizing the OpenCV library, it provides functionality for initializing camera parameters, transforming 3D points to 2D image space, calibrating cameras, and interfacing with real webcams. Additionally, it offers tools for working with ChArUco boards—a type of fiducial marker used in computer vision.
  - `Camera.h`: Header for Camera.cpp.
- **README.md**: This document.
- **Robot**:
  - `Robot.cpp`: [Description about what Robot.cpp does]
  - `Robot.h`: Header for Robot.cpp.
- **Serial**:
  - `Serial.cpp`: [Description about what Serial.cpp does]
  - `Serial.h`: Header for Serial.cpp.
- `cvui.h`: [Short description about cvui.h]
- **stdafx**:
  - `stdafx.cpp`
  - `stdafx.h`
- **uArm**:
  - `uArm.cpp`: [Description about what uArm.cpp does]
  - `uArm.h`: Header for uArm.cpp.

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

# IMU-Bird-Flight-Monitoring
### Made by: Isaac Medina

## Overview
This repository contains firmware, data processing scripts, and analysis tools for an inertial motion unit (IMU)-based system designed to track bird flight in three dimensions. The system supports both the BNO055 and LSM6DS3 IMUs, with data collected via embedded microcontrollers and transmitted wirelessly for post-processing and visualization.

The goal of this project is to reconstruct accurate 3D motion paths of free-moving birds using lightweight, onboard IMU sensors. This enables researchers to monitor flight behavior and subtle movement patterns in a minimally invasive manner, with potential applications in ethology, biomechanics, and assessing environmental stress.

## Project Context
This project is part of a larger, ongoing research initiative that I joined during the Spring 2025 semester, focused on understanding physiological stress responses in birds. The broader research aims to develop a noninvasive framework for monitoring both behavioral and physiological markers of stress in birds. By combining IMU-derived flight data with additional physiological data (e.g., heart rate, hormone levels), the project contributes to a growing field of ecophysiological monitoring technologies.

## Features
- Real-time data collection from 6-DoF and 9-DoF IMUs (LSM6DS3 and BNO055) --> 9-DoF BNO055 for final implementation
- Embedded system design for data logging and wireless transmission via BLE using nRF52832 (Coming soon...)
- Python scripts for filtering, integration, and 3D trajectory reconstruction
- Kalman Filter and Zero-Velocity Update (ZUPT) algorithms to reduce drift
- Modular architecture to support sensor replacement and system scaling

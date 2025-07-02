# IMU-Bird-Flight-Monitoring

## Contributors
- **Isaac Medina** - Firmware development, digital post processing, and visualization
  - Tufts University Undergraduate - Class of 2027
  - Electrical Engineering

## Institutional Affiliations
This project is conducted as part of ongoing research in the **Sonkusale Research Lab** in collaboration with the **Romero Lab** <br>

**Tufts University** <br>
Department of Electrical and Computer Engineering <br>
Halligan Lab <br>
Medford/Somerville Campus

## Ackowledgements
I would like to thank **Surya Varchasvi**, Research Fellow at Tufts University, for his mentorship and guidance throughout this project.  

I would also like to acknowledge **Rachel Riccio**, Biology PhD candidate at Tufts University, for her collaboration and ongoing support in applying this technology to real-world behavioral and physiological studies in birds.

Special thanks to [Dr. Sameer Sonkusale](https://engineering.tufts.edu/ece/people/faculty/sameer-sonkusale) for the opportunity to work in the lab.


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

## Software Design - Digital Post Processing
![Block Diagram](Images/Kalman-Filter-Block-Diagram-Bird-Project.png)

## Repository Structure
- **BNO055**
  - Data Collection Code ESP32 --> Contains ESP32 Arduino code for logging output data from BNO055 9-DoF IMU
  - Datasets --> Contains raw CSV datalogs
  - Figures --> Contains output figures from data analysis, such as position plots and state outputs
  - BNO055_Position_v1.py --> Initial simple algorithm for BNO055
  - BNO055_Position_v2.py --> First Kalman filter and ZUPT algorithm implementation
  
- **Images**
  - Contains various images related to the project, such as block diagrams, PCB designs, and miscellaneous pictures.

- **LSM6DS3**
  - Data Collection Code ESP32 --> Contains ESP32 Arduino code for logging output data from LSM6DS3 6-DoF IMU
  - Datasets --> Contains raw CSV datalogs
  - Figures --> Contains output figures from data analysis, such as position plots and state outputs
  - IMU_to_Position1.py --> First algorithm code for POC


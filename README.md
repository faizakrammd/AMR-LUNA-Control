<img width="1080" height="1080" alt="image" src="https://github.com/user-attachments/assets/57ad086c-b841-499f-9cf1-fd10ffc8c27c" /># AMR-LUNA-Control
Control algorithms for mobile robot named "LUNA". 
# LUNA Control Algorithms

Link: https://flywheelaerospace.com/product/luna-mobile-robot-kit/

A collection of control algorithms, estimation techniques, and sensor fusion experiments developed for the **LUNA Differential Drive Mobile Robot**.

The objective of this repository is to build a mobile robot from first principles by implementing every algorithm from scratch, understanding the underlying mathematics, and validating the results on real hardware.

Unlike projects that rely heavily on high-level robotics frameworks, this repository focuses on the core algorithms that enable autonomous mobile robots.

---

## Objectives

* Understand mobile robot control from first principles.
* Develop robust control algorithms for differential drive robots.
* Implement sensor fusion for accurate state estimation.
* Validate theoretical models through real-world experiments.
* Build a reusable foundation for autonomous robotics research.

---

# Hardware

* ESP32
* MPU6050 IMU
* N20 DC Motors with Encoders
* TB6612FNG Motor Driver
* VL53L0X Time-of-Flight Sensor
* Differential Drive Robot Platform

---

# Topics Covered

## Motion & Kinematics

* Differential Drive Kinematics
* Robot Coordinate Frames
* Wheel Calibration
* Velocity Estimation
* Dead Reckoning
* Odometry

---

## Control Algorithms

* Open Loop Control
* Bang-Bang Control
* PID Control
* Heading Lock
* Rotation Control
* Straight Line Tracking
* Trajectory Tracking
* Feedforward Control
* Adaptive Control
* Model Predictive Control (Planned)

---

## Sensor Fusion

* Complementary Filter
* Gyroscope Bias Estimation
* Noise Filtering
* 1D Kalman Filter
* Extended Kalman Filter (Planned)
* Encoder + IMU Fusion

---

## Localization

* Pose Estimation
* Error Propagation
* Dead Reckoning Drift
* State Estimation

---

## Navigation

* Obstacle Avoidance
* Finite State Machine
* Potential Fields
* Occupancy Grid Mapping
* BFS
* A*
* Local Navigation
* Goal Navigation

---

## Multi-Robot Systems (Planned)

* Leader-Follower
* Consensus Algorithms
* Distributed Control
* Formation Control
* Cooperative Localization
* Swarm Robotics

# Philosophy

Every algorithm in this repository follows the same development process:

1. Mathematical formulation
2. System modelling
3. Algorithm implementation
4. Experimental validation
5. Performance evaluation
6. Real-world testing on LUNA

The emphasis is on understanding how robotics algorithms work internally rather than treating them as black-box libraries.

---
# License

This repository is intended for educational, research, and learning purposes.

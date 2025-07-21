# Visual Odometry with ZED 2 Stereo Camera

This repository contains the development of an algorithm for trajectory reconstruction of the ZED 2 stereo camera. The main goal is to estimate the camera motion by processing stereo images and testing different combinations of feature detectors and descriptors.

## Objectives

- Implement visual odometry pipeline for the ZED 2 stereo camera
- Compare the performance of various feature detectors (e.g., SIFT, ORB, FAST)
- Test different feature descriptors for robustness and accuracy
- Reconstruct the 3D trajectory of the stereo camera based on matched features
- Evaluate accuracy against known initial and final position

## Contents

- Scripts for feature detection and description
- Stereo matching and triangulation modules
- Pose estimation from stereo correspondences
- Visualization tools for trajectory plotting

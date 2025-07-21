# Visual Odometry with ZED 2 Stereo Camera

This repository contains the development of an algorithm for trajectory reconstruction of the ZED 2 stereo camera. The main goal is to estimate the camera motion by processing stereo images and testing different combinations of feature detectors and descriptors.

## Objectives

- Implement visual odometry pipeline for the ZED 2 stereo camera
- Compare the performance of various feature detectors (e.g., SIFT, ORB, FAST)
- Test different feature descriptors for robustness and accuracy
- Reconstruct the 3D trajectory of the stereo camera based on matched features
- Evaluate accuracy against ground truth if available

## Repository Contents

- Scripts for feature detection and description
- Stereo matching and triangulation modules
- Pose estimation from stereo correspondences
- Visualization tools for trajectory plotting

## Requirements

- MATLAB or Python environment (specify which if you want)
- OpenCV (if Python)
- ZED SDK (optional, if working with real camera data)

## Usage

Clone the repository and run the main script. Adjust the detector and descriptor parameters to test different combinations.

## License

MIT License


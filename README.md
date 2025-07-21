
---

## 📁 [`visual-odometry`](https://github.com/FrancescoGriguol/visual-odometry)

```markdown
# Visual Odometry with Monocular Event Camera

This repository contains MATLAB code for visual odometry using event-based camera data (e.g., DAVIS346). It integrates corner detection, optical flow tracking, and pose estimation from essential matrix decomposition.

## Features

- Corner detection using Arc* algorithm on event-based time surfaces
- Feature tracking via Lucas-Kanade optical flow
- Selection of suitable keyframes for baseline triangulation
- Pose estimation using monocular geometry (Essential matrix + relative pose)
- Triangulation of 3D landmarks

## Files Overview

- `detectCorners.m`: Applies Arc* to extract corners from event frames.
- `trackFeatures.m`: Tracks features between keyframes using optical flow.
- `selectBaseline.m`: Chooses a keyframe pair for triangulation.
- `estimatePose.m`: Computes relative pose from tracked correspondences.
- `triangulatePoints.m`: Reconstructs 3D points from keyframe pairs.

## Requirements

- MATLAB R2021a or later
- Computer Vision Toolbox

## Future Work

- Integration with IMU data for full visual-inertial odometry
- Bundle adjustment and global trajectory optimization

## License

This project is licensed under the MIT License.

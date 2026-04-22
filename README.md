# RealSense Robotics Pipeline

A five-phase robotics perception pipeline built with an Intel RealSense D435 depth camera, ROS2 (Robot Operating System 2) Humble, and Python. Starting from raw camera data and ending with robot-ready grasp candidates. Each phase builds directly on the last.

**Hardware:** Intel RealSense D435  
**Platform:** Windows 11 + WSL2 (Windows Subsystem for Linux 2, Ubuntu 22.04)  
**Framework:** ROS2 Humble  

---

## The Pipeline 

```
Camera Feed
    │
    ▼
Phase 0 — Raw depth + RGB frames, point cloud visualisation
    │
    ▼
Phase 1 — ROS2 pipeline: camera data published as ROS2 topics
    │
    ▼
Phase 2 — Occupancy Grid: 3D scene converted to 2D navigation map
    │
    ▼
Phase 3 — Visual SLAM: persistent 3D map built by moving camera
    │
    ▼
Phase 4 — 6-DOF (Degrees of Freedom) Pose Estimation: YOLO (You Only Look Once) detection + PCA (Principal Component Analysis) orientation
    │
    ▼
Phase 5 — Grasp Detection: gripper approach directions + scoring
```

---

## Phases

| Phase | Topic | Key Tools |
|-------|-------|-----------|
| [Phase 0 — Camera Basics](phase0_camera/) | Depth imaging, point clouds | pyrealsense2, Open3D |
| [Phase 1 — ROS2 Setup](phase1_ros2_setup/) | ROS2 pipeline, WSL2, USB passthrough | ROS2 Humble, usbipd-win |
| [Phase 2 — Occupancy Grid](phase2_occupancy_grid/) | 2D navigation map from depth | ROS2, nav_msgs |
| [Phase 3 — Visual SLAM](phase3_slam/) | 3D map building, loop closure | RTAB-Map, rtabmap_ros |
| [Phase 4 — Pose Estimation](phase4_pose_estimation/) | 6-DOF object poses | YOLO, PCA, RViz2 (ROS Visualisation tool) |
| [Phase 5 — Grasp Detection](phase5_grasp_detection/) | Robot grasp candidates | Grasp scoring, RViz2 markers |

---

## Why This Project Exists

Most robotics perception demos stop at object detection — drawing a bounding box and calling it done. This project goes further: from detecting an object, to knowing exactly where it is in 3D space, to computing *how a robot gripper should approach and grab it*.

Each phase is a real sub-problem in robotics:
- **Mapping** (Phase 2, 3) — a robot needs to know what space is free before it can move
- **Localisation** (Phase 3) — it needs to know where it is within that map
- **Perception** (Phase 4) — it needs to identify objects and their 3D poses
- **Manipulation** (Phase 5) — it needs to know how to physically interact with objects

This pipeline covers all four, using only a single depth camera and a laptop.

---

## Hardware & Software Requirements

### Hardware
- Intel RealSense D435 (or D415/D435i)
- A laptop or desktop with a USB 3.0 port

### Software
- Windows 11 with WSL2 enabled
- Ubuntu 22.04 inside WSL2
- ROS2 Humble
- Python 3.10+

Full setup instructions are in [Phase 1 — ROS2 Setup](phase1_ros2_setup/).

---

## Results Summary

| Phase | What You See |
|-------|-------------|
| Phase 0 | Live depth feed + 3D point cloud updating in real time |
| Phase 1 | RGB and depth streams visible as ROS2 topics in rqt_image_view |
| Phase 2 | 2D occupancy map updating as camera pans across the scene |
| Phase 3 | Full 3D point cloud map building as camera explores the room |
| Phase 4 | RGB axis arrows showing object orientation live in RViz2 |
| Phase 5 | Green approach arrows + red finger spheres on graspable objects |

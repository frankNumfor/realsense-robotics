# Phase 4 — 6-DOF Pose Estimation: Where Objects Are and How They're Oriented

## Overview

Knowing that an object exists in the scene is not enough for a robot to interact with it. The robot needs to know the object's full **pose**: where it is in 3D space, and how it is oriented. Together, these six values; three for position (X, Y, Z) and three for orientation (roll, pitch, yaw), are called a **6-DOF pose** (six degrees of freedom).

In this phase we combine YOLO object detection with depth data and PCA-based orientation estimation to compute a live 6-DOF pose for every detected object, published as a standard ROS2 message.

---

## What is 6-DOF?

A rigid object in 3D space has six independent ways it can move:

**3 translational (position):**
- X — left/right
- Y — up/down
- Z — forward/backward

**3 rotational (orientation):**
- Roll — rotation around the forward axis (tilting sideways)
- Pitch — rotation around the left/right axis (nodding)
- Yaw — rotation around the up/down axis (turning left/right)

Knowing all six values precisely is what a robot arm needs to reach out and touch a specific face of an object. A 2D bounding box from a camera tells you roughly where an object is in the image, but not how far away it is, and not how it is oriented. 6-DOF pose solves both.

---

## Step 1: Object Detection with YOLO

**YOLO (You Only Look Once)** is a family of real-time object detection neural networks. Unlike two-stage detectors that first propose regions then classify them, YOLO processes the entire image in a single forward pass, making it fast enough for real-time use.

YOLO divides the image into a grid. Each grid cell predicts:
- Whether an object's centre falls in that cell
- Bounding box coordinates (x, y, width, height)
- Class probabilities across all 80 COCO classes

The result is a set of bounding boxes, each with a class label and confidence score.

We use **YOLOv8s** (small variant of YOLOv8) for a balance of speed and accuracy:

```python
model = YOLO('yolov8s.pt')
results = model(color_frame, conf=0.3, verbose=False)
```

### COCO Dataset

YOLOv8 is trained on **COCO (Common Objects in COntext)** — a dataset of ~330,000 images with 80 object categories. The 80 classes include everyday objects: bottle, cup, chair, laptop, book, phone, person, etc.

If an object is not in these 80 classes (e.g., a pen, a screwdriver), YOLO will not detect it. This is a hard constraint of using a pretrained model without fine-tuning.

---

## Step 2: Getting 3D Position from Depth

Once YOLO gives us a bounding box (x1, y1, x2, y2), we use the depth image to back-project every pixel inside that box to 3D space, exactly as in Phase 0:

```python
z = depth_crop / 1000.0
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```

This gives us a **point cloud of the object**; all the 3D points belonging to that object's region.

The object's **3D position** is the centroid (mean) of this point cloud:

```python
position = pts.mean(axis=0)   # (X, Y, Z) in metres
```

---

## Step 3: Orientation via PCA

Position tells us where the object is. Orientation tells us how it is rotated. We estimate orientation using **PCA (Principal Component Analysis)**.

### What PCA Does

Given a set of 3D points (our object point cloud), PCA finds the three directions along which the points are most spread out. These are called the **principal axes**.

For a bottle lying on its side, the points are most spread along the bottle's long axis. For a book flat on a table, they are most spread along the book's two face directions. The principal axes describe the object's natural orientation in 3D space.

### The Maths

1. **Centre the data:** subtract the mean so the cloud is centred at the origin
2. **Compute the covariance matrix:** a 3×3 matrix that captures how spread-out the points are in each direction and how those spreads correlate
   ```python
   centred = pts - pts.mean(axis=0)
   cov = np.cov(centred.T)   # 3×3 matrix
   ```
3. **Eigendecomposition:** find the eigenvectors and eigenvalues of the covariance matrix
   ```python
   eigvals, eigvecs = np.linalg.eigh(cov)
   ```
   The eigenvectors are the principal axes. The eigenvalues are how much variance (spread) exists along each axis. Sorting eigenvectors by descending eigenvalue gives axes ordered from most-spread to least-spread.

4. **Form a rotation matrix:** the three principal axes form the columns of a 3×3 rotation matrix, describing how the object's frame is rotated relative to the camera frame.

5. **Convert to quaternion:** rotation matrices are converted to quaternions for the ROS2 message.

### What is a Quaternion?

Rotation in 3D can be represented as rotating by some angle θ around some axis (nx, ny, nz). A quaternion encodes this as four numbers: (x, y, z, w), where:
- (x, y, z) = sin(θ/2) × (nx, ny, nz)
- w = cos(θ/2)

Quaternions avoid **gimbal lock** (a problem with Euler angles where two rotation axes align and a degree of freedom is lost) and are easier to interpolate and compose. ROS2 uses quaternions for all orientation data.

---

## Orientation Wobble: A Known Limitation

The **position** estimate is stable; the centroid of hundreds of depth points is robust to individual noisy measurements.

The **orientation** estimate wobbles between frames. This is expected:
- Depth sensors have per-pixel noise of a few millimetres
- PCA finds the axes of maximum spread, which are sensitive to the shape of the point cloud boundary
- Thin objects (books, laptops) have one very small eigenvalue — the axis perpendicular to the thin face — making that axis direction highly sensitive to noise

Stabilising orientation would require temporal filtering (averaging orientations over multiple frames) or a more sophisticated shape model. For the purposes of this portfolio, the wobble is a known limitation that is clearly understood and explained.

---

## Output: ROS2 PoseArray

The node publishes to three topics:

```
/object_poses     ← geometry_msgs/PoseArray  (position + orientation per object)
/object_markers   ← visualization_msgs/MarkerArray  (RGB axis arrows in RViz2 — ROS Visualisation tool)
/pose_estimation/image  ← sensor_msgs/Image  (YOLO annotated frame)
```

`PoseArray` is a standard ROS2 message; a robot arm controller can subscribe directly to `/object_poses` and use the data to plan reach motions, without knowing anything about how the poses were computed.

---

## Visualisation in RViz2

The node publishes coloured axis arrows as RViz2 markers for each detected object:
- **Red arrow** — X axis (principal axis 1, direction of most spread)
- **Green arrow** — Y axis (principal axis 2)
- **Blue arrow** — Z axis (principal axis 3, direction of least spread — usually perpendicular to the object face)

The RViz2 config file (`pose_estimation.rviz`) is pre-configured with `camera_link` as the Fixed Frame and displays both the marker arrows and the annotated camera image.

---

## Setup

```bash
pip3 install ultralytics scipy
```

**Terminal 1 — Camera:**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**Terminal 2 — Pose estimation node:**
```bash
ros2 run depth_nav pose_estimation_node
```

**Terminal 3 — Visualise:**
```bash
rviz2 -d ~/pose_estimation.rviz
```

**Terminal 4 — Annotated image feed:**
```bash
rqt_image_view
# Select /pose_estimation/image
```

---

## Results

![Live 6-DOF pose estimation on a robotics lab scene](phase4_pose.gif)

The GIF shows the live system running in a robotics lab. YOLO detects multiple objects simultaneously (cup, keyboard, mouse, tv) and the terminal logs the full 6-DOF pose for each on every frame:

```
cup:      pos=(0.03, 0.01, 0.88)m  rpy=(-52.8, 82.2, 125.2)deg
keyboard: pos=(0.20, 0.07, 1.09)m  rpy=(-90.2, 78.7, 178.0)deg
mouse:    pos=(-0.17, 0.15, 0.65)m  rpy=(134.7, -74.3, 138.4)deg
tv:       pos=(0.42, -0.16, 0.99)m  rpy=(-27.5, -74.6, 21.3)deg
```

Position values (X, Y, Z in metres) remain stable across frames. Orientation values (roll, pitch, yaw in degrees) fluctuate slightly due to depth noise on the object surface — a known and understood limitation of geometry-based pose estimation.

---

## Key Takeaways

- 6-DOF = 3 position values + 3 orientation values — the complete description of an object's pose in 3D space
- YOLO detects objects in 2D; depth data extends those detections into 3D
- PCA finds the natural axes of an object's point cloud — its principal axes describe its orientation
- Eigendecomposition of the covariance matrix is the mathematical core of PCA
- Quaternions are the standard rotation representation in robotics
- Position is stable; orientation wobbles due to depth noise — a known, understood limitation

---

*Next: [Phase 5 — Grasp Detection](../phase5_grasp_detection/) — computing where and how a robot gripper should approach each object.*

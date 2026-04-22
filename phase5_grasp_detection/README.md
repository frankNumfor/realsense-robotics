# Phase 5 — Grasp Detection: Computing Robot Gripper Approaches

## Overview

Phase 4 told us where objects are and how they are oriented. Phase 5 answers the next question: **how should a robot gripper approach and grab each object?**

This is grasp detection; computing a set of candidate grasp poses, each describing a specific position, approach direction, and gripper opening width, and ranking them by quality.

---

## The Grasp Problem

A robotic grasp is not just "move to the object." The gripper needs to:

1. **Approach** from a direction that avoids collisions with the table, nearby objects, and the gripper's own body
2. **Orient** so the two fingers land on opposite sides of the object
3. **Open** to the right width — not so wide it misses, not so narrow it can't close around the object

Finding valid grasps is an active research area. Industrial solutions use deep learning trained on millions of grasp examples. Our approach is geometry-based: we use the object's point cloud shape to reason about where fingers can make stable contact.

---

## The GraspCandidate

Each candidate grasp is described by five values:

| Field | Description |
|-------|-------------|
| `position` | 3D point where the gripper palm should be placed (metres) |
| `approach` | Unit vector pointing from gripper toward object (the direction of closing) |
| `binormal` | Unit vector along the axis connecting the two finger tips |
| `gripper_width` | Distance between fingers at contact (metres) |
| `score` | Quality score 0–1 (higher is better) |

The `approach` and `binormal` vectors together fully define the gripper's orientation. The third axis is their cross product.

---

## Generating Candidates: Sampling Around the Principal Axis

We generate multiple candidate grasps by rotating potential approach directions around the object's principal axis (the direction of maximum spread, computed via PCA — Principal Component Analysis — as in Phase 4).

```python
angles = np.linspace(0, np.pi, n_candidates, endpoint=False)

for angle in angles:
    rot      = Rotation.from_rotvec(principal * angle)
    approach = rot.apply(eigvecs[:, 1])       # rotate the second principal axis
    binormal = np.cross(approach, principal)   # finger-to-finger direction
```

For each angle, we get a different approach direction. Rotating through π (180°) covers all geometrically distinct orientations — π to 2π would just repeat them with fingers swapped.

### Gripper Width Filter

For each candidate, we project all object points onto the binormal axis (the finger-to-finger direction) and compute the span:

```python
projections   = centred @ binormal
gripper_width = projections.max() - projections.min()
```

If this width falls outside the gripper's physical range (2cm – 10cm), the candidate is discarded:

```python
if gripper_width < gripper_min or gripper_width > gripper_max:
    continue
```

This is why large objects like chairs and dining tables never produce grasp candidates — their cross-section along any approach direction exceeds 10cm. A parallel gripper physically cannot span them.

---

## Scoring Candidates

Each surviving candidate is scored on three criteria, weighted and summed:

```python
score = 0.4 * approach_score + 0.3 * width_score + 0.3 * vertical_score
```

### Approach Alignment Score (40%)

Measures how closely the approach direction aligns with the camera's forward axis (Z):

```python
camera_forward = np.array([0., 0., 1.])
approach_score = abs(np.dot(approach, camera_forward))
```

Approaching along the camera's line of sight means the arm approaches from roughly in front of the object — the most natural direction for a camera-mounted arm.

### Gripper Width Fit Score (30%)

Rewards candidates where the gripper width is close to the midpoint of its range:

```python
width_mid   = (gripper_max + gripper_min) / 2   # 6cm
width_score = 1.0 - abs(gripper_width - width_mid) / width_mid
```

A gripper at the extreme of its range (barely open or barely closed) has less force margin. A comfortable mid-range grasp is more robust.

### Vertical Approach Bonus (30%)

Rewards candidates that approach from above (downward into the scene):

```python
up             = np.array([0., -1., 0.])   # up in camera coords is -Y
vertical_score = max(0., np.dot(approach, up))
```

Approaching from above a table is almost always collision-free — the arm does not need to navigate around other objects. Side approaches risk collisions with neighbouring items.

---

## Visualisation: Green Arrows and Red Spheres

The node publishes `visualization_msgs/MarkerArray` to `/grasp_markers` (visualised in RViz2 — ROS Visualisation tool). For each candidate:

- **Green arrow** — starts at the gripper approach position, points toward the object along the approach vector. The arrow shows the direction the gripper would travel to close on the object.
- **Two red spheres** — placed at the two finger tip positions, one on each side of the object at `±gripper_width/2` along the binormal axis.

```python
tip = cand.position + cand.approach * 0.08
# Arrow from cand.position to tip

fp = cand.position + cand.binormal * sign * cand.gripper_width / 2
# Red sphere at each finger position
```

All candidates are visualised, not just the best one — so you can see the full range of proposed grasps and understand why the top-scored one was selected.

---

## Terminal Output

Every frame, the best grasp for each detected object is logged:

```
bottle: best grasp at (-0.07, 0.34, 1.54)m  width=5.7cm  score=0.71
book:   best grasp at (-0.31, 0.46, 1.48)m  width=9.8cm  score=0.22
```

The bottle scores higher (0.71) than the book (0.22) because its narrower width fits the gripper range more comfortably, and the approach direction aligns better with the scoring criteria.

---

## Limitations and Next Steps

This system computes **where** and **how** to grasp — the planning step. For a real robot to execute the grasp, several more components are needed:

- **Robot arm** — a physical manipulator to carry the gripper
- **Motion planning** — computing a collision-free joint trajectory from current arm pose to grasp pose (e.g., MoveIt2)
- **Force/torque sensing** — detecting when fingers make contact and how hard they are pressing
- **Closed-loop control** — adjusting the grasp in real time based on sensor feedback

The grasp quality scoring is also simplified. Real grasp planners model contact mechanics, friction, and stability more rigorously (e.g., GraspIt!, Dex-Net).

---

## Setup

```bash
pip3 install ultralytics scipy
```

**Terminal 1 — Camera:**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**Terminal 2 — Grasp detection node:**
```bash
ros2 run depth_nav grasp_detection_node
```

**Terminal 3 — Visualise markers in RViz2:**
```bash
rviz2 -d ~/grasp_detection.rviz
```

**Terminal 4 — Annotated image feed:**
```bash
rqt_image_view
# Select /grasp_detection/image
```

---

## Results

![Live grasp detection on a robotics lab scene](phase5_grasp.gif)

The GIF shows the system running in a robotics lab, detecting graspable objects and computing best grasp candidates in real time. Detected objects with their best grasp poses:

```
cup:      best grasp at (0.03, 0.06, 0.88)m  width=9.1cm  score=0.52
keyboard: best grasp at (0.16, 0.07, 1.13)m  width=8.9cm  score=0.24
mouse:    best grasp at (0.16, 0.07, 1.12)m  width=9.0cm  score=0.24
```

- **Cup** scores highest — compact, graspable cross-section at 9.1cm width
- **Keyboard** and **mouse** score lower — wider, flatter surfaces with fewer high-quality approach angles
- **TV and chair** produce no candidates — too wide for the 2–10 cm gripper range, which is correct behaviour

The gripper width filter (2–10 cm) reflects a physical constraint of a real parallel gripper. Objects wider than 10 cm simply cannot be grasped with that tool, so the node correctly rejects them rather than producing invalid candidates.

---

## Key Takeaways

- Grasp detection extends pose estimation: from "where is the object" to "how do I grab it"
- Candidates are generated by sampling approach directions around the object's principal axis
- The gripper width filter eliminates physically impossible grasps before scoring
- Scoring balances three factors: approach alignment, width comfort, and vertical preference
- The output is robot-arm-ready: position + approach vector + gripper width is directly actionable
- Real robot execution would additionally require motion planning, force sensing, and closed-loop control

---

*This is the final phase of the pipeline. See the [main README](../README.md) for the full project overview.*

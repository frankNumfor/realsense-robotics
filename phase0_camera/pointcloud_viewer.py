import numpy as np
import open3d as o3d
from realsense_depth import DepthCamera

# Depth range filter (meters) — ignore points outside this range
MIN_DEPTH = 0.2
MAX_DEPTH = 5.0


def depth_to_pointcloud(depth_image, intrinsics, color_image=None):
    """
    Back-project a depth image into a 3D point cloud.

    Parameters
    ----------
    depth_image  : (H, W) uint16 numpy array — depth in millimetres
    intrinsics   : rs.intrinsics object from DepthCamera
    color_image  : (H, W, 3) BGR numpy array (optional) — used for colouring points

    Returns
    -------
    points : (N, 3) float32 — XYZ in metres
    colors : (N, 3) float32 in [0,1] — RGB, or None if no color_image provided
    """
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy

    h, w = depth_image.shape

    # Build pixel coordinate grids — shape (H, W)
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)

    # Convert depth from millimetres to metres
    z = depth_image.astype(np.float32) / 1000.0

    # Back-project using pinhole camera model
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy

    # Stack into (H, W, 3) then flatten to (H*W, 3)
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)

    # Filter: remove invalid depth (z=0) and out-of-range points
    mask = (z.reshape(-1) >= MIN_DEPTH) & (z.reshape(-1) <= MAX_DEPTH)
    points = points[mask]

    colors = None
    if color_image is not None:
        # Convert BGR -> RGB, normalise to [0, 1]
        rgb = color_image[:, :, ::-1].astype(np.float32) / 255.0
        colors = rgb.reshape(-1, 3)[mask]

    return points, colors


def main():
    dc = DepthCamera()

    # Create Open3D visualiser in non-blocking mode
    vis = o3d.visualization.Visualizer()
    vis.create_window("Point Cloud — RealSense D435", width=1280, height=720)

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # Coordinate frame axes (X=red, Y=green, Z=blue) — camera origin
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    vis.add_geometry(axes)

    reset_done = False

    print("Streaming point cloud  —  press Q in the Open3D window to quit")
    print("Controls: left-click drag = rotate | scroll = zoom | right-click drag = pan")

    while True:
        ret, depth_image, color_image = dc.get_frame()
        if not ret:
            continue

        points, colors = depth_to_pointcloud(depth_image, dc.intrinsics, color_image)

        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.update_geometry(pcd)

        if not vis.poll_events():
            break

        vis.update_renderer()

        # Reset view once we have a full frame of points
        if not reset_done and len(points) > 1000:
            vis.reset_view_point(True)
            reset_done = True

    dc.release()
    vis.destroy_window()


if __name__ == "__main__":
    main()
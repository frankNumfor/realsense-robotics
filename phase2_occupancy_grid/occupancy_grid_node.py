#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # --- parameters ---------------------------------------------------
        self.declare_parameter('resolution',     0.05)  # metres per cell
        self.declare_parameter('grid_width',     200)   # cells  (10 m)
        self.declare_parameter('grid_height',    200)   # cells  (10 m)
        self.declare_parameter('min_height',     0.1)   # metres above ground
        self.declare_parameter('max_height',     1.5)   # metres above ground
        self.declare_parameter('camera_height',  1.0)   # camera height above ground

        self.res     = self.get_parameter('resolution').value
        self.g_w     = self.get_parameter('grid_width').value
        self.g_h     = self.get_parameter('grid_height').value
        self.min_h   = self.get_parameter('min_height').value
        self.max_h   = self.get_parameter('max_height').value
        self.cam_h   = self.get_parameter('camera_height').value

        self.intrinsics = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=1
        )

        self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            self.info_cb, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_cb, sensor_qos
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        self.get_logger().info('OccupancyGridNode ready')

    
    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
            }
            self.get_logger().info(
                f'Intrinsics received  fx={self.intrinsics["fx"]:.1f}'
            )

    
    def depth_cb(self, msg):
        if self.intrinsics is None:
            return

        # ROS depth image is uint16, values in millimetres
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        )
        points = self.backproject(depth)
        grid_msg = self.build_grid(points, msg.header)
        self.grid_pub.publish(grid_msg)

    
    def backproject(self, depth):
        """Convert depth image to (N,3) point cloud in camera frame."""
        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        cx, cy = self.intrinsics['cx'], self.intrinsics['cy']

        h, w = depth.shape
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = depth.astype(np.float32) / 1000.0   # mm -> metres
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy

        pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        # Keep only valid depth readings (0.2 m – 8 m)
        mask = (z.reshape(-1) > 0.2) & (z.reshape(-1) < 8.0)
        return pts[mask]

   
    def build_grid(self, pts, header):
        """
        Project points onto a 2D grid.

        Camera frame convention (ROS): X = right, Y = down, Z = forward.
        Real-world height above floor  =  camera_height - Y
        We use X (lateral) and Z (forward) as the 2D map axes.
        """
        grid = np.full((self.g_h, self.g_w), -1, dtype=np.int8)  # -1 = unknown

        cx_g = self.g_w // 2   # camera sits at lateral centre of grid

        # ---- mark free space (any valid depth reading) ----------------
        xi = (pts[:, 0] / self.res + cx_g).astype(int)
        zi = (pts[:, 2] / self.res).astype(int)
        in_bounds = (xi >= 0) & (xi < self.g_w) & (zi >= 0) & (zi < self.g_h)
        grid[zi[in_bounds], xi[in_bounds]] = 0   # free

        # ---- mark obstacles (points at obstacle height) ---------------
        real_h = self.cam_h - pts[:, 1]
        obs_mask = (real_h > self.min_h) & (real_h < self.max_h)
        obs = pts[obs_mask]

        oxi = (obs[:, 0] / self.res + cx_g).astype(int)
        ozi = (obs[:, 2] / self.res).astype(int)
        in_bounds_o = (
            (oxi >= 0) & (oxi < self.g_w) & (ozi >= 0) & (ozi < self.g_h)
        )
        grid[ozi[in_bounds_o], oxi[in_bounds_o]] = 100  # occupied

        # ---- build ROS message ----------------------------------------
        out = OccupancyGrid()
        out.header        = header
        out.header.frame_id = 'camera_link'
        out.info.resolution = self.res
        out.info.width      = self.g_w
        out.info.height     = self.g_h
        out.info.origin.position.x  = -cx_g * self.res  # left edge of grid
        out.info.origin.position.y  = 0.0
        out.info.origin.position.z  = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.flatten().tolist()
        return out



def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

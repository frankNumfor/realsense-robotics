#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from ultralytics import YOLO
except ImportError:
    raise RuntimeError("Run: pip3 install ultralytics")


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        self.declare_parameter('confidence',    0.3)
        self.declare_parameter('min_points',    50)
        self.declare_parameter('camera_height', 1.0)

        self.conf    = self.get_parameter('confidence').value
        self.min_pts = self.get_parameter('min_points').value
        self.cam_h   = self.get_parameter('camera_height').value

        self.intrinsics  = None
        self.depth_image = None

        self.model = YOLO('yolov8s.pt')
        self.get_logger().info('YOLO model loaded')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=1
        )

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.info_cb, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_cb, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.color_cb, sensor_qos
        )

        self.poses_pub   = self.create_publisher(PoseArray,   '/object_poses',   10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.image_pub   = self.create_publisher(Image, '/pose_estimation/image', 10)

        self.get_logger().info('PoseEstimationNode ready')

    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
            }
            self.get_logger().info('Intrinsics received')

    def depth_cb(self, msg):
        self.depth_image = np.frombuffer(
            msg.data, dtype=np.uint16
        ).reshape(msg.height, msg.width).copy()

    def color_cb(self, msg):
        if self.intrinsics is None or self.depth_image is None:
            return

        color_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        ).copy()

        # ROS publishes RGB, OpenCV/YOLO expects BGR
        color = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)

        results = self.model(color, conf=self.conf, verbose=False)

        # Publish annotated image as ROS2 topic
        annotated = results[0].plot()
        img_msg          = Image()
        img_msg.header   = msg.header
        img_msg.height   = annotated.shape[0]
        img_msg.width    = annotated.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.step     = annotated.shape[1] * 3
        img_msg.data     = annotated.tobytes()
        self.image_pub.publish(img_msg)

        pose_array   = PoseArray()
        marker_array = MarkerArray()
        pose_array.header.frame_id = 'camera_link'
        pose_array.header.stamp    = msg.header.stamp

        marker_id = 0
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                class_name = self.model.names[int(box.cls[0])]

                pts = self.box_to_points(x1, y1, x2, y2)
                if pts is None or len(pts) < self.min_pts:
                    continue

                position    = pts.mean(axis=0)
                orientation = self.estimate_orientation(pts)

                pose_msg = self.make_pose(position, orientation, msg.header)
                pose_array.poses.append(pose_msg.pose)

                markers = self.make_axes_markers(
                    position, orientation, class_name, msg.header, marker_id
                )
                marker_array.markers.extend(markers)
                marker_id += 4

                rpy = Rotation.from_quat(orientation).as_euler('xyz', degrees=True)
                self.get_logger().info(
                    f'{class_name}: '
                    f'pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})m  '
                    f'rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})deg'
                )

        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(marker_array)

    def box_to_points(self, x1, y1, x2, y2):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']

        depth_crop = self.depth_image[y1:y2, x1:x2].astype(np.float32)
        h, w = depth_crop.shape
        if h == 0 or w == 0:
            return None

        u = np.arange(x1, x1 + w, dtype=np.float32)
        v = np.arange(y1, y1 + h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = depth_crop / 1000.0
        mask = (z > 0.1) & (z < 5.0)
        if mask.sum() < self.min_pts:
            return None

        x = (uu[mask] - cx) * z[mask] / fx
        y = (vv[mask] - cy) * z[mask] / fy
        return np.stack([x, y, z[mask]], axis=-1)

    def estimate_orientation(self, pts):
        centred          = pts - pts.mean(axis=0)
        cov              = np.cov(centred.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        axes             = eigvecs[:, ::-1]
        axes[:, 2]       = np.cross(axes[:, 0], axes[:, 1])
        rot              = Rotation.from_matrix(axes)
        return rot.as_quat()

    def make_pose(self, position, quaternion, header):
        msg                     = PoseStamped()
        msg.header              = header
        msg.header.frame_id     = 'camera_link'
        msg.pose.position.x     = float(position[0])
        msg.pose.position.y     = float(position[1])
        msg.pose.position.z     = float(position[2])
        msg.pose.orientation.x  = float(quaternion[0])
        msg.pose.orientation.y  = float(quaternion[1])
        msg.pose.orientation.z  = float(quaternion[2])
        msg.pose.orientation.w  = float(quaternion[3])
        return msg

    def make_axes_markers(self, position, quaternion, label, header, mid):
        rot    = Rotation.from_quat(quaternion).as_matrix()
        colors = [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
        markers = []

        for i, (axis, color) in enumerate(zip(rot.T, colors)):
            m                 = Marker()
            m.header          = header
            m.header.frame_id = 'camera_link'
            m.ns              = 'pose_axes'
            m.id              = mid + i
            m.type            = Marker.ARROW
            m.action          = Marker.ADD
            m.scale.x         = 0.008
            m.scale.y         = 0.015
            m.color.a         = 1.0
            m.color.r, m.color.g, m.color.b = color
            m.lifetime.sec    = 1

            start = Point(x=float(position[0]),
                          y=float(position[1]),
                          z=float(position[2]))
            end   = Point(x=float(position[0] + axis[0] * 0.1),
                          y=float(position[1] + axis[1] * 0.1),
                          z=float(position[2] + axis[2] * 0.1))
            m.points = [start, end]
            markers.append(m)

        t                 = Marker()
        t.header          = header
        t.header.frame_id = 'camera_link'
        t.ns              = 'pose_labels'
        t.id              = mid + 3
        t.type            = Marker.TEXT_VIEW_FACING
        t.action          = Marker.ADD
        t.pose.position.x = float(position[0])
        t.pose.position.y = float(position[1]) - 0.1
        t.pose.position.z = float(position[2])
        t.pose.orientation.w = 1.0
        t.scale.z         = 0.05
        t.color.a         = 1.0
        t.color.r = t.color.g = t.color.b = 1.0
        t.text            = label
        t.lifetime.sec    = 1
        markers.append(t)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

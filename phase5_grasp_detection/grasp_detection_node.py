#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from ultralytics import YOLO
except ImportError:
    raise RuntimeError("Run: pip3 install ultralytics")


class GraspCandidate:
    """Represents a single grasp candidate."""
    def __init__(self, position, approach, binormal, gripper_width, score):
        self.position      = position
        self.approach      = approach
        self.binormal      = binormal
        self.gripper_width = gripper_width
        self.score         = score


class GraspDetectionNode(Node):
    def __init__(self):
        super().__init__('grasp_detection_node')

        self.declare_parameter('confidence',     0.3)
        self.declare_parameter('min_points',     50)
        self.declare_parameter('gripper_max',    0.10)
        self.declare_parameter('gripper_min',    0.02)
        self.declare_parameter('standoff',       0.05)
        self.declare_parameter('num_candidates', 8)

        self.conf     = self.get_parameter('confidence').value
        self.min_pts  = self.get_parameter('min_points').value
        self.g_max    = self.get_parameter('gripper_max').value
        self.g_min    = self.get_parameter('gripper_min').value
        self.standoff = self.get_parameter('standoff').value
        self.n_cands  = self.get_parameter('num_candidates').value

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

        self.markers_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self.image_pub   = self.create_publisher(Image, '/grasp_detection/image', 10)

        self.get_logger().info('GraspDetectionNode ready')

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
        color = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)

        results = self.model(color, conf=self.conf, verbose=False)

        annotated        = results[0].plot()
        img_msg          = Image()
        img_msg.header   = msg.header
        img_msg.height   = annotated.shape[0]
        img_msg.width    = annotated.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.step     = annotated.shape[1] * 3
        img_msg.data     = annotated.tobytes()
        self.image_pub.publish(img_msg)

        marker_array = MarkerArray()
        marker_id    = 0

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                class_name = self.model.names[int(box.cls[0])]

                pts = self.box_to_points(x1, y1, x2, y2)
                if pts is None or len(pts) < self.min_pts:
                    continue

                candidates = self.generate_grasps(pts)
                if not candidates:
                    continue

                best = candidates[0]
                self.get_logger().info(
                    f'{class_name}: best grasp at '
                    f'({best.position[0]:.2f}, {best.position[1]:.2f}, '
                    f'{best.position[2]:.2f})m  '
                    f'width={best.gripper_width*100:.1f}cm  '
                    f'score={best.score:.2f}'
                )

                for cand in candidates:
                    markers = self.make_grasp_markers(cand, msg.header, marker_id)
                    marker_array.markers.extend(markers)
                    marker_id += 3

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

    def generate_grasps(self, pts):
        """
        Generate grasp candidates by sampling approach directions around
        the object's principal axis and scoring each one.
        """
        centroid = pts.mean(axis=0)
        centred  = pts - centroid
        cov      = np.cov(centred.T)

        eigvals, eigvecs = np.linalg.eigh(cov)
        order    = np.argsort(eigvals)[::-1]
        eigvecs  = eigvecs[:, order]
        principal = eigvecs[:, 0]

        candidates = []
        angles = np.linspace(0, np.pi, self.n_cands, endpoint=False)

        for angle in angles:
            rot      = Rotation.from_rotvec(principal * angle)
            approach = rot.apply(eigvecs[:, 1])
            approach = approach / np.linalg.norm(approach)

            binormal = np.cross(approach, principal)
            if np.linalg.norm(binormal) < 1e-6:
                continue
            binormal = binormal / np.linalg.norm(binormal)

            projections   = centred @ binormal
            gripper_width = projections.max() - projections.min()

            if gripper_width < self.g_min or gripper_width > self.g_max:
                continue

            position = centroid - approach * self.standoff

            # Score: approach alignment + gripper width fit + vertical bonus
            camera_forward = np.array([0., 0., 1.])
            approach_score = abs(np.dot(approach, camera_forward))

            width_mid    = (self.g_max + self.g_min) / 2
            width_score  = 1.0 - abs(gripper_width - width_mid) / width_mid

            up             = np.array([0., -1., 0.])
            vertical_score = max(0., np.dot(approach, up))

            score = 0.4 * approach_score + 0.3 * width_score + 0.3 * vertical_score

            candidates.append(GraspCandidate(
                position, approach, binormal, gripper_width, score
            ))

        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates

    def make_grasp_markers(self, cand, header, mid):
        """Green arrow = approach direction. Red spheres = finger positions."""
        markers = []

        arrow                 = Marker()
        arrow.header          = header
        arrow.header.frame_id = 'camera_link'
        arrow.ns              = 'grasp_approach'
        arrow.id              = mid
        arrow.type            = Marker.ARROW
        arrow.action          = Marker.ADD
        arrow.scale.x         = 0.008
        arrow.scale.y         = 0.015
        arrow.color.a         = 0.9
        arrow.color.g         = 1.0
        arrow.lifetime.sec    = 1

        tip = cand.position + cand.approach * 0.08
        arrow.points = [
            Point(x=float(cand.position[0]),
                  y=float(cand.position[1]),
                  z=float(cand.position[2])),
            Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))
        ]
        markers.append(arrow)

        for i, sign in enumerate([-1, 1]):
            fp     = cand.position + cand.binormal * sign * cand.gripper_width / 2
            sphere = Marker()
            sphere.header          = header
            sphere.header.frame_id = 'camera_link'
            sphere.ns              = 'grasp_fingers'
            sphere.id              = mid + 1 + i
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = float(fp[0])
            sphere.pose.position.y = float(fp[1])
            sphere.pose.position.z = float(fp[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
            sphere.color.a = 0.8
            sphere.color.r = 1.0
            sphere.lifetime.sec = 1
            markers.append(sphere)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = GraspDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Life-Rover YOLO Detector ROS2 Node.

DaBai 카메라의 RGB/Depth 토픽을 구독해서:
  1. RGB 영상에 YOLO11 추론
  2. 사람 검출 시 박스 좌표 + 신뢰도 출력
  3. depth 토픽으로 거리 측정
  4. 결과를 콘솔에 출력
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('YOLO Detector starting...')

        self.bridge = CvBridge()

        self.get_logger().info('Loading YOLO11n model...')
        self.model = YOLO('yolo11n.pt')

        # 워밍업
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model(dummy, classes=[0], conf=0.4, verbose=False)
        self.get_logger().info('YOLO ready')

        self.latest_depth = None

        self.rgb_sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.rgb_callback, 10,
        )
        self.depth_sub = self.create_subscription(
            Image, '/depth_cam/depth/image_raw', self.depth_callback, 10,
        )

        self.frame_count = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info('Subscribed to camera topics. Waiting for frames...')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().warn(f'depth conversion failed: {e}')

    def rgb_callback(self, msg):
        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'rgb conversion failed: {e}')
            return

        results = self.model(color, classes=[0], conf=0.40, verbose=False)

        boxes_info = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy.tolist()
                conf = float(box.conf[0].cpu().numpy())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                distance_m = None
                if self.latest_depth is not None:
                    h, w = self.latest_depth.shape[:2]
                    rgb_h, rgb_w = color.shape[:2]
                    dcx = int(cx * w / rgb_w)
                    dcy = int(cy * h / rgb_h)
                    roi = self.latest_depth[
                        max(0, dcy-2):dcy+3,
                        max(0, dcx-2):dcx+3
                    ]
                    valid = roi[(roi > 200) & (roi < 10000)]
                    if valid.size > 0:
                        distance_m = float(np.median(valid)) / 1000.0

                boxes_info.append({
                    'bbox': (x1, y1, x2, y2),
                    'conf': conf,
                    'distance_m': distance_m,
                })

        self.frame_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed >= 2.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'FPS: {fps:.1f}, persons: {len(boxes_info)}'
            )
            for i, b in enumerate(boxes_info):
                dist_str = f'{b["distance_m"]:.2f}m' if b['distance_m'] else 'N/A'
                self.get_logger().info(
                    f'  person {i}: conf={b["conf"]:.2f}, '
                    f'distance={dist_str}, bbox={b["bbox"]}'
                )
            self.frame_count = 0
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

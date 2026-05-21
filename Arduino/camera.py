# camera.py
"""
USB Camera 추상화 계층.

현 상태: DaBai DCW의 RGB만 UVC로 받음. Depth는 가짜로 시뮬레이션.
(나중에 pyorbbecsdk 도입 시 DepthCamera 클래스 추가 후 분기.)
"""
import numpy as np
import cv2

CAMERA_INDEX = 1

try:
    import pyrealsense2 as rs
    _HAS_RS = True
except ImportError:
    _HAS_RS = False


class DepthCamera:
    """
    Intel RealSense D4xx 시리즈용. 현재 환경에선 사용 안 됨.
    나중에 RealSense 장비로 교체 시 자동 활용 가능.
    """
    def __init__(self, width=640, height=480, fps=30):
        if not _HAS_RS:
            raise RuntimeError("pyrealsense2 미설치")
        self.width, self.height, self.fps = width, height, fps

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(cfg)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

        for _ in range(10):
            try:
                self.pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                pass

        print(f"[camera] RealSense opened ({width}x{height}@{fps}fps, depth_scale={self.depth_scale})")

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError:
            return None, None
        aligned = self.align.process(frames)
        c = aligned.get_color_frame()
        d = aligned.get_depth_frame()
        if not c or not d:
            return None, None
        color = np.asanyarray(c.get_data())
        depth_raw = np.asanyarray(d.get_data()).astype(np.uint16)
        if abs(self.depth_scale - 0.001) > 1e-6:
            depth_mm = (depth_raw.astype(np.float32) * self.depth_scale * 1000).astype(np.uint16)
        else:
            depth_mm = depth_raw
        return color, depth_mm

    def release(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


class UvcDepthCamera:
    """
    DaBai DCW의 RGB 스트림을 일반 UVC로 받음.
    Depth는 진짜가 아니라 시뮬레이션 값.
    (나중에 pyorbbecsdk로 진짜 depth 받게 교체 예정)
    """
    def __init__(self, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"카메라 인덱스 {CAMERA_INDEX}를 열 수 없어. "
                "장치 관리자 / 다른 앱 점유 여부 확인."
            )
        self.width, self.height = width, height

        for _ in range(3):
            self.cap.read()

        print(f"[camera] UVC camera (index={CAMERA_INDEX}) opened ({width}x{height})")

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None, None
        # 시뮬레이션 depth: 중앙으로 갈수록 가까운 그라데이션
        h, w = frame.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        cx, cy = w // 2, h // 2
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        depth = (1500 + dist * 3).astype(np.uint16)
        return frame, depth

    def release(self):
        self.cap.release()


def open_camera(width=640, height=480, fps=30):
    """
    RealSense 우선 시도, 실패 시 UVC 카메라(DaBai RGB 또는 웹캠) fallback.
    """
    if _HAS_RS:
        try:
            return DepthCamera(width, height, fps)
        except Exception as e:
            print(f"[camera] RealSense 열기 실패 → UVC fallback: {e}")
    return UvcDepthCamera(width, height, fps)
import os

# ── 카메라 설정 ─────────────────────────────────────
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30


MODEL_PATH    = os.environ.get("MODEL_PATH", "yolo11n.pt")
CONF_THRESH   = 0.40
PERSON_CLASS  = 0   # COCO 'person' 클래스 ID


TCP_HOST = "0.0.0.0"   # 모든 인터페이스에서 수신
TCP_PORT = 9999

# ── JPEG 송신 품질 (1~100, 낮을수록 대역폭↓ 화질↓) ──
JPEG_QUALITY = 70

# ── 알림 임계값 ─────────────────────────────────────

ALERT_MIN_CONF = 0.55
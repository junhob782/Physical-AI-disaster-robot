import cv2
import numpy as np
from ultralytics import YOLO


class PersonDetector:
    def __init__(self, model_path="yolo11n.pt", conf=0.4, person_class=0):
        print(f"[detector] loading model: {model_path}")
        self.model = YOLO(model_path)
        self.conf = conf
        self.person_class = person_class

        # 워밍업 (첫 추론이 5~10초 걸리는 걸 미리 해소)
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model(dummy, classes=[person_class], conf=conf, verbose=False)
        print("[detector] warmup done")

    def detect(self, frame_bgr, depth_mm=None):
        """
        반환: list of dict
          [{"x1","y1","x2","y2","conf","cx","cy", "distance_m"(있을 때만)}, ...]
        """
        results = self.model(
            frame_bgr,
            classes=[self.person_class],
            conf=self.conf,
            verbose=False,
        )
        boxes_out = []
        if not results:
            return boxes_out

        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return boxes_out

        h, w = frame_bgr.shape[:2]
        for box in r.boxes:
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy.tolist()
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            confv = float(box.conf[0].cpu().numpy())

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            entry = {
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": round(confv, 3),
                "cx": cx, "cy": cy,
            }

            
            # 5x5 영역의 median을 써서 강건하게 거리 추정
            if depth_mm is not None:
                roi = depth_mm[
                    max(0, cy - 2): cy + 3,
                    max(0, cx - 2): cx + 3
                ]
                valid = roi[roi > 0]
                if valid.size > 0:
                    entry["distance_m"] = round(float(np.median(valid)) / 1000.0, 2)

            boxes_out.append(entry)
        return boxes_out

    @staticmethod
    def draw(frame_bgr, boxes):
        """frame_bgr에 박스와 라벨을 그려넣음 (in-place)."""
        for b in boxes:
            x1, y1, x2, y2 = b["x1"], b["y1"], b["x2"], b["y2"]
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)

            label = f"Person {b['conf']:.2f}"
            if "distance_m" in b:
                label += f"  {b['distance_m']:.1f}m"

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(frame_bgr, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(
                frame_bgr, label, (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA
            )
        return frame_bgr
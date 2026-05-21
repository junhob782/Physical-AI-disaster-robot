import cv2
import json
import signal
import sys
import threading
import time
from collections import deque

import config
from camera import open_camera
from detector import PersonDetector
from server import FrameServer

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception:
    rclpy = None
    Node = object
    String = None


SENSOR_TOPIC = "/sensor_data"


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("vision_sensor_subscriber")
        self._latest = None
        self._lock = threading.Lock()
        self.create_subscription(String, SENSOR_TOPIC, self._on_sensor_data, 10)

    def _on_sensor_data(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}

        with self._lock:
            self._latest = data

    def latest(self):
        with self._lock:
            return self._latest


class SensorBridge:
    def __init__(self):
        self.node = None
        self.thread = None
        self.enabled = False

    def start(self):
        if rclpy is None:
            print("[sensor] ROS2 rclpy not available. sensor_data disabled.")
            return

        rclpy.init(args=None)
        self.node = SensorSubscriber()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()
        self.enabled = True
        print(f"[sensor] subscribing to {SENSOR_TOPIC}")

    def latest(self):
        if not self.enabled or self.node is None:
            return None
        return self.node.latest()

    def stop(self):
        if not self.enabled:
            return
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()
        print("[sensor] stopped")


def draw_sensor_overlay(color, sensor_data):
    if not sensor_data:
        return

    sensor_text = json.dumps(sensor_data, ensure_ascii=False)[:80]
    cv2.putText(
        color,
        f"sensor: {sensor_text}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )


def main():
    print("[main] starting Life-Rover Vision Server")

    cam = open_camera(
        width=config.CAMERA_WIDTH,
        height=config.CAMERA_HEIGHT,
        fps=config.CAMERA_FPS,
    )
    det = PersonDetector(
        model_path=config.MODEL_PATH,
        conf=config.CONF_THRESH,
        person_class=config.PERSON_CLASS,
    )
    srv = FrameServer(config.TCP_HOST, config.TCP_PORT)
    srv.start()

    sensor_bridge = SensorBridge()
    sensor_bridge.start()

    stop_flag = {"v": False}

    def on_sigint(sig, frame):
        stop_flag["v"] = True

    signal.signal(signal.SIGINT, on_sigint)

    fps_window = deque(maxlen=30)
    frame_id = 0
    print("[main] running. ESC or 'q' to quit.")

    try:
        while not stop_flag["v"]:
            t0 = time.perf_counter()

            color, depth = cam.read()
            if color is None:
                continue

            boxes = det.detect(color, depth_mm=depth)
            det.draw(color, boxes)

            dt = time.perf_counter() - t0
            fps_window.append(1.0 / max(dt, 1e-6))
            fps_avg = sum(fps_window) / len(fps_window)

            alert = any(b["conf"] >= config.ALERT_MIN_CONF for b in boxes)
            sensor_data = sensor_bridge.latest()
            draw_sensor_overlay(color, sensor_data)

            ok, jpeg = cv2.imencode(
                ".jpg",
                color,
                [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY],
            )
            if not ok:
                continue

            meta = {
                "ts": int(time.time() * 1000),
                "frame_id": frame_id,
                "fps": round(fps_avg, 1),
                "n_person": len(boxes),
                "alert": alert,
                "boxes": boxes,
                "sensor": sensor_data,
                "img_w": color.shape[1],
                "img_h": color.shape[0],
            }

            srv.broadcast(jpeg.tobytes(), meta)
            frame_id += 1

            cv2.imshow("LifeRover Preview (q=quit)", color)
            key = cv2.waitKey(1)
            if key != -1:
                key &= 0xFF
                if key == ord("q") or key == 27:
                    print(f"[main] quit key detected: {key}")
                    break

            if frame_id % 100 == 0 and frame_id > 0:
                print(
                    f"[main] frame {frame_id}, fps={fps_avg:.1f}, "
                    f"clients={srv.n_clients()}, sensor={sensor_data is not None}"
                )

    except Exception as e:
        print(f"[main] error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
    finally:
        sensor_bridge.stop()
        srv.stop()
        cam.release()
        cv2.destroyAllWindows()
        print("[main] bye")


if __name__ == "__main__":
    main()

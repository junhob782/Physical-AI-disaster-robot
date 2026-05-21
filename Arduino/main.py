import cv2
import time
import signal
import sys
from collections import deque

import config
from camera import open_camera
from detector import PersonDetector
from server import FrameServer


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

    # Ctrl+C로 깔끔히 종료할 수 있도록
    stop_flag = {"v": False}
    def on_sigint(sig, frame):
        stop_flag["v"] = True
    signal.signal(signal.SIGINT, on_sigint)

    fps_window = deque(maxlen=30)
    frame_id = 0
    print("[main] running. ESC 또는 'q' 키로 종료.")

    try:
        while not stop_flag["v"]:
            t0 = time.perf_counter()

            color, depth = cam.read()
            if color is None:
                continue

            boxes = det.detect(color, depth_mm=depth)

            # 박스 그리기 (color 이미지에 직접 그림)
            det.draw(color, boxes)

            # JPEG 인코딩
            ok, jpeg = cv2.imencode(
                ".jpg",
                color,
                [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY],
            )
            if not ok:
                continue

            # FPS 계산 (이동평균)
            dt = time.perf_counter() - t0
            fps_window.append(1.0 / max(dt, 1e-6))
            fps_avg = sum(fps_window) / len(fps_window)

            # 알림 판정
            alert = any(b["conf"] >= config.ALERT_MIN_CONF for b in boxes)

            # 메타데이터 구성
            meta = {
                "ts": int(time.time() * 1000),
                "frame_id": frame_id,
                "fps": round(fps_avg, 1),
                "n_person": len(boxes),
                "alert": alert,
                "boxes": boxes,
                "img_w": color.shape[1],
                "img_h": color.shape[0],
            }

            # 전송
            srv.broadcast(jpeg.tobytes(), meta)
            frame_id += 1

            # 로컬 프리뷰
            cv2.imshow("LifeRover Preview (q=quit)", color)
            key = cv2.waitKey(1)
            # waitKey는 키 안 눌리면 -1 반환. & 0xFF 하면 255가 되어버려서
            # 우연한 키 해석 위험이 있음. -1 체크를 먼저.
            if key != -1:
                key &= 0xFF
                if key == ord('q') or key == 27:
                    print(f"[main] 종료 키 감지: {key}")
                    break

            # 디버그: 100프레임마다 상태 출력
            if frame_id % 100 == 0 and frame_id > 0:
                print(f"[main] frame {frame_id}, fps={fps_avg:.1f}, clients={srv.n_clients()}")

    except Exception as e:
        print(f"[main] error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        srv.stop()
        cam.release()
        cv2.destroyAllWindows()
        print("[main] bye")


if __name__ == "__main__":
    main()
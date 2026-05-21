import socket
import struct
import json
import cv2
import numpy as np
import sys


def recv_exact(sock, n):
    """소켓에서 정확히 n바이트를 읽음. 부족하면 더 받아가며 채움."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("서버가 연결을 끊었어")
        buf += chunk
    return buf


def main(host="127.0.0.1", port=9999):
    print(f"[client] connecting to {host}:{port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("[client] connected")

    frame_count = 0
    try:
        while True:
            # 1) 헤더 4바이트: 전체 페이로드 길이
            header = recv_exact(sock, 4)
            total_len = struct.unpack(">I", header)[0]

            # 2) 페이로드 받기
            payload = recv_exact(sock, total_len)

            # 3) 메타 길이 + 메타 JSON + JPEG로 분해
            meta_len = struct.unpack(">H", payload[:2])[0]
            meta_bytes = payload[2:2 + meta_len]
            jpeg_bytes = payload[2 + meta_len:]

            meta = json.loads(meta_bytes.decode("utf-8"))

            # 4) JPEG를 OpenCV 이미지로 디코딩
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                print("[client] JPEG 디코딩 실패")
                continue

            frame_count += 1
            # 10프레임마다 메타 출력 (너무 자주 찍히면 어지러움)
            if frame_count % 10 == 0:
                print(
                    f"[client] frame {meta['frame_id']}: "
                    f"persons={meta['n_person']}, "
                    f"fps={meta['fps']}, "
                    f"alert={meta['alert']}, "
                    f"jpeg_size={len(jpeg_bytes)} bytes"
                )

            # 알림 발생 시 즉시 출력
            if meta.get("alert"):
                print(f"  ⚠ ALERT! boxes: {meta['boxes']}")

            cv2.imshow("Client View (q=quit)", img)
            key = cv2.waitKey(1)
            if key != -1 and (key & 0xFF) in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        print("\n[client] interrupted")
    except Exception as e:
        print(f"[client] error: {e}")
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print(f"[client] total frames received: {frame_count}")


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    main(host=host)
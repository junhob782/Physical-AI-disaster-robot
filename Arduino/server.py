import socket
import struct
import json
import threading


class FrameServer:
    def __init__(self, host="0.0.0.0", port=9999):
        self.host, self.port = host, port
        self._server_sock = None
        self._clients = []
        self._lock = threading.Lock()
        self._running = False
        self._accept_thread = None

    def start(self):
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.host, self.port))
        self._server_sock.listen(5)
        self._server_sock.settimeout(1.0)  # accept를 1초마다 깨움 (종료 응답성)
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        print(f"[server] listening on {self.host}:{self.port}")

    def _accept_loop(self):
        while self._running:
            try:
                conn, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(2.0)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append(conn)
            print(f"[server] client connected: {addr}  (총 {len(self._clients)}대)")

    def broadcast(self, jpeg_bytes: bytes, meta: dict) -> int:
        """모든 클라이언트에 한 프레임을 전송. 반환값: 성공한 클라이언트 수."""
        meta_bytes = json.dumps(meta, ensure_ascii=False).encode("utf-8")
        meta_len = len(meta_bytes)
        if meta_len > 0xFFFF:
            raise ValueError(f"meta JSON이 너무 큼: {meta_len} bytes")

        total_len = 2 + meta_len + len(jpeg_bytes)
        header = struct.pack(">IH", total_len, meta_len)
        payload = header + meta_bytes + jpeg_bytes

        dead = []
        sent = 0
        with self._lock:
            for conn in self._clients:
                try:
                    conn.sendall(payload)
                    sent += 1
                except Exception:
                    dead.append(conn)
            # 끊어진 클라이언트 정리
            for d in dead:
                try: d.close()
                except: pass
                self._clients.remove(d)
        return sent

    def n_clients(self) -> int:
        with self._lock:
            return len(self._clients)

    def stop(self):
        self._running = False
        with self._lock:
            for c in self._clients:
                try: c.close()
                except: pass
            self._clients.clear()
        if self._server_sock:
            try: self._server_sock.close()
            except: pass
        print("[server] stopped")
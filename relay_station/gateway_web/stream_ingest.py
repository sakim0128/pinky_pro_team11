#!/usr/bin/env python3
"""
Tablet & Mobile Phone Camera Single Ingest & Zero-Lag Buffer Manager
- 태블릿 및 스마트폰 IP 카메라(IP Webcam MJPEG, DroidCam 등) 인제스트
- 동적 URL 갱신(update_target_url) 및 모바일 브라우저 직접 업로드(ingest_direct_frame) 지원
- 로컬 네트워크 IP 카메라 자동 탐색(auto_scan_cameras) 지원
- CAP_PROP_BUFFERSIZE = 1 및 스레드 기반 0-lag 프레임 갱신
- 네트워크 단절 시 자동 재연결 및 실시간 안내 플레이스홀더 제공
"""

import os
os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"

import time
import socket
import threading
import concurrent.futures
import cv2
# OpenCV 로그 끄기 — Ubuntu 24.04 의 apt python3-opencv(4.6) 에는 cv2.setLogLevel 이 없다(cv2.utils.logging 에만 있다)
try:
    cv2.setLogLevel(0)
except AttributeError:
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except Exception:            # noqa: BLE001 — 로그 레벨은 편의일 뿐, 없어도 돈다
        pass
import numpy as np


class TabletStreamIngest:
    def __init__(self, stream_url="", fallback_url=""):
        # ⭐⭐ 주소를 **지어내지 않는다.** 예전 기본값(`198.51.100.2:8080` /
        #    `100.64.0.6:8080`)은 둘 다 죽어 있었고 - 포트부터 틀렸다. 태블릿은
        #    앱 `:18086` / 중계 `:18082` 를 쓴다. 그 죽은 주소로 15초마다 재접속하며
        #    5,293줄을 쌓았고, 그 로그가 **진짜 접속 실패를 가렸다**(2026-09-12 실측).
        #    pull 로 당겨오는 소스는 `configs/video_sources.json` + MjpegPuller 가
        #    후보 목록까지 들고 제대로 한다. 여기는 **push 싱크**가 본래 역할이다.
        self.primary_url = stream_url
        self.fallback_url = fallback_url
        self.current_url = self.primary_url

        self.cap = None
        self.is_running = False
        self.is_connected = False
        self.thread = None
        self._force_reconnect = threading.Event()
        self._last_direct_upload_stamp = 0.0

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = 0.0
        self._latest_jpeg = None
        self._fps = 0.0

        # 초기 플레이스홀더 프레임 생성
        self._update_placeholder("휴대폰/태블릿 카메라 연결 대기 중...")

    def _create_placeholder(self, message="Waiting for Stream..."):
        img = np.zeros((720, 1280, 3), dtype=np.uint8)
        img[:] = (24, 28, 36)

        # 박스
        cv2.rectangle(img, (180, 180), (1100, 540), (45, 55, 72), -1)
        cv2.rectangle(img, (180, 180), (1100, 540), (0, 180, 255), 2)

        # 텍스트
        cv2.putText(img, "MOBILE & TABLET CAMERA GATEWAY", (230, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Status: {message}", (230, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Target URL: {self.primary_url}", (230, 380),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 190, 205), 1, cv2.LINE_AA)
        cv2.putText(img, "Tip: open /camera_streamer on the phone (Hershey font is ASCII-only)", (230, 430),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 200, 80), 1, cv2.LINE_AA)

        # 시간 표시
        time_str = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(img, f"Gateway Time: {time_str}", (230, 485),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 200, 100), 1, cv2.LINE_AA)
        return img

    def _update_placeholder(self, message):
        frame = self._create_placeholder(message)
        stamp = time.time()
        _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        with self._lock:
            self._latest_frame = frame
            self._latest_stamp = stamp
            self._latest_jpeg = jpeg.tobytes()

    def update_target_url(self, new_url):
        """동적으로 카메라 스트림 URL을 갱신하고 즉시 재연결을 시도"""
        new_url = (new_url or "").strip()
        if not new_url:
            return False
        if not (new_url.startswith("http://") or new_url.startswith("https://") or new_url.startswith("rtsp://")):
            new_url = f"http://{new_url}"

        # 경로가 생략된 경우 기본값 /video 보정
        if new_url.count('/') == 2 and ':' in new_url.split('/')[-1]:
            new_url = f"{new_url}/video"

        print(f"[StreamIngest] Switching Target Camera URL to: {new_url}")
        self.primary_url = new_url
        self.current_url = new_url
        self._force_reconnect.set()
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.is_connected = False
        self._update_placeholder(f"Reconnecting to {new_url}...")
        return True

    def ingest_direct_frame(self, frame_bytes):
        """휴대폰 브라우저 및 자체개발 앱(HTTP POST)으로부터 프레임을 수신하여 버퍼 갱신"""
        try:
            # 1. JSON Base64 페이로드 지원
            if frame_bytes.startswith(b'{'):
                try:
                    import json, base64
                    data = json.loads(frame_bytes.decode('utf-8'))
                    b64_str = data.get('image') or data.get('frame') or data.get('data') or ''
                    if ',' in b64_str:
                        b64_str = b64_str.split(',', 1)[1]
                    frame_bytes = base64.b64decode(b64_str)
                except Exception:
                    pass

            # 2. Multipart/form-data인 경우 JPEG 매직 바이트 추출
            if b'\xff\xd8\xff' in frame_bytes:
                start = frame_bytes.find(b'\xff\xd8\xff')
                end = frame_bytes.rfind(b'\xff\xd9')
                if end > start:
                    frame_bytes = frame_bytes[start:end+2]

            stamp = time.time()
            np_arr = np.frombuffer(frame_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with self._lock:
                    self._latest_frame = frame
                    self._latest_stamp = stamp
                    self._latest_jpeg = frame_bytes
                    self.is_connected = True
                    self.current_url = "📱 자체개발 앱 실시간 송출"
                    self._last_direct_upload_stamp = stamp
                return True
        except Exception as e:
            print(f"[StreamIngest] Direct frame error: {e}")
        return False

    def auto_scan_cameras(self, subnet_prefix="198.51.100", ip_range=(2, 40)):
        """서브넷 내 활성 IP 카메라 포트(8080, 4747, 8081, 18081) 고속 스캔"""
        candidates = []
        ports = [8080, 4747, 8081, 18081]

        def probe(ip, port):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.3)
            try:
                if sock.connect_ex((ip, port)) == 0:
                    path = "/video" if port != 4747 else "/video"
                    return {"ip": ip, "port": port, "url": f"http://{ip}:{port}{path}"}
            except Exception:
                pass
            finally:
                sock.close()
            return None

        tasks = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=60) as executor:
            for last_octet in range(ip_range[0], ip_range[1] + 1):
                ip = f"{subnet_prefix}.{last_octet}"
                for port in ports:
                    tasks.append(executor.submit(probe, ip, port))
            for future in concurrent.futures.as_completed(tasks):
                res = future.result()
                if res:
                    candidates.append(res)

        return candidates

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.is_running = False
        self._force_reconnect.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()

    def _try_open(self, url):
        cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
        return None

    def _capture_loop(self):
        last_stat_time = time.time()
        frame_count = 0
        last_retry_log = 0.0
        idle_noted = False        # 주소 없음을 한 번만 말한다 - 그것도 반복되면 소음이다

        while self.is_running:
            # 휴대폰 브라우저에서 직접 프레임이 송출되고 있는 경우 OpenCV 캡처 대기
            if time.time() - self._last_direct_upload_stamp < 3.0:
                time.sleep(0.5)
                continue

            # ⭐ 주소가 없으면 아무 데도 안 붙는다. "못 붙었다" 와 "붙을 데를 안 줬다" 는
            #    다른 사실이고, 둘을 같은 재접속 로그로 뭉개면 앞의 것을 못 본다.
            if not self.primary_url:
                if not idle_noted:
                    self._update_placeholder(
                        "pull 주소 미설정 - 이 소스는 앱 push 전용이다")
                    print("[Mobile Ingest] pull 주소가 없다 - 재접속하지 않는다. "
                          "push(/api/camera/upload)만 받는다. "
                          "당겨올 소스는 configs/video_sources.json 에 둔다.")
                    idle_noted = True
                self._force_reconnect.wait(timeout=2.0)
                continue
            idle_noted = False

            target_url = self.primary_url
            self._update_placeholder(f"Connecting to {target_url}...")

            now = time.time()
            if now - last_retry_log >= 5.0:
                print(f"[Mobile Ingest] Waiting for Mobile/Tablet at {target_url} ... (Auto-reconnecting)")
                last_retry_log = now

            self._force_reconnect.clear()
            cap = self._try_open(target_url)

            if cap is None:
                # fallback 확인
                if self.fallback_url and self.fallback_url != target_url:
                    cap = self._try_open(self.fallback_url)
                    if cap is not None:
                        target_url = self.fallback_url

            if cap is None:
                # 1.5초 대기하되 강제 재연결 신호 발생 시 즉시 반응
                self._force_reconnect.wait(timeout=1.5)
                continue

            self.cap = cap
            self.current_url = target_url
            self.is_connected = True
            print(f"[MobileStreamIngest] Successfully connected to {target_url}")

            while self.is_running and not self._force_reconnect.is_set():
                # 직접 업로드가 시작되면 캡처 루프 양보
                if time.time() - self._last_direct_upload_stamp < 3.0:
                    break

                ret, frame = self.cap.read()
                if not ret:
                    print(f"[MobileStreamIngest] Stream disconnected from {target_url}")
                    break

                stamp = time.time()
                frame_count += 1

                now = time.time()
                if now - last_stat_time >= 1.0:
                    self._fps = frame_count / (now - last_stat_time)
                    frame_count = 0
                    last_stat_time = now

                _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                jpeg_bytes = jpeg.tobytes()

                with self._lock:
                    self._latest_frame = frame
                    self._latest_stamp = stamp
                    self._latest_jpeg = jpeg_bytes

            self.is_connected = False
            if self.cap:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

            time.sleep(0.5)

    def get_latest_frame(self):
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0, False, 0.0
            return self._latest_frame.copy(), self._latest_stamp, self.is_connected, self._fps

    def get_latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg, self._latest_stamp, self.is_connected

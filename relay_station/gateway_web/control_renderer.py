#!/usr/bin/env python3
"""
Legacy Control Screen compatibility provider.

2026-09-21 UI renewal:
- 운영 관제는 static/index.html 의 브라우저 Canvas가 담당한다.
- 이 모듈은 /control_feed 하위호환을 위해 정적 안내 프레임만 제공한다.
- 상시 10 FPS OpenCV 렌더 스레드는 실행하지 않는다(CPU 회수).
- 주행 명령/CLICK-TO-MOVE 안내를 렌더하지 않는다.
"""

import os
import time
import math
import threading
import cv2
import numpy as np
import yaml


class ControlScreenRenderer:
    def __init__(self, map_yaml_path="$HOME/my_map.yaml"):
        self.map_yaml_path = map_yaml_path
        self.map_source = None   # 실제로 읽은 파일 경로, 합성이면 "synthetic"
        self.map_img = None
        self.resolution = 0.05
        self.origin = [-2.325, -2.100, 0.0]
        self.map_h = 400
        self.map_w = 400

        # 로봇 1, Gazebo Sim, 로봇 2 상태 관리
        # 🔴 좌표 기본값은 **None** 이다. 0.0 으로 두면 "아직 안 왔다" 와 "원점에
        #    서 있다" 가 같은 숫자가 되고, robot2 처럼 아예 지어낸 값(1.5/-0.8/1.57)이
        #    `/api/status.robots` 를 타고 화면에 실측처럼 찍힌다(2026-09-18 감사).
        #    같은 파일의 `get_discrepancy` 는 이미 "못 쟀으면 못 쟀다고 한다" 로
        #    고쳐져 있었고 이 경로만 빠져 있었다.
        self.robots = {
            'robot1': {'x': None, 'y': None, 'yaw': None, 'color': (0, 140, 255), 'trail': [], 'name': 'Pinky #1 (Real)', 'active': True},
            'gazebo_sim': {'x': None, 'y': None, 'yaw': None, 'color': (255, 230, 0), 'trail': [], 'name': 'Gazebo Sim', 'active': True},
            'robot2': {'x': None, 'y': None, 'yaw': None, 'color': (200, 100, 255), 'trail': [], 'name': 'Pinky #2 (Sub)', 'active': False}
        }

        # 정밀 보정된 미션 경유지 1, 2, 3 (Gazebo & SLAM 맵 100% 일치 좌표)
        self.waypoints = [
            {'id': 1, 'name': 'Origin (1)', 'x': 0.0, 'y': -0.1, 'color': (0, 255, 100)},
            {'id': 2, 'name': 'Point 2', 'x': -1.4, 'y': -0.2, 'color': (255, 180, 0)},
            {'id': 3, 'name': 'Point 3', 'x': 1.3, 'y': -0.3, 'color': (200, 100, 255)},
        ]

        # 실시간 네비게이션 목표 및 상태
        self.current_goal = None
        self.nav_status = {'state': 'IDLE', 'mission': '', 'remaining_dist': 0.0}

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_jpeg = None
        self._load_map()

        # 2026-09-21 W-3: 운영 UI는 브라우저 Canvas로 이관됐다.
        # /control_feed 때문에 코어를 계속 점유하지 않도록 상시 렌더 스레드를 띄우지 않는다.
        self.is_running = False
        self.thread = None
        self._make_legacy_placeholder()

    def _make_legacy_placeholder(self):
        """`/control_feed` 직접 접근자에게 브라우저 UI 이관 사실만 알린다.

        상태/좌표는 `/api/status` + `static/index.html` 이 그린다. 이 프레임은
        실시간 상태처럼 보이면 안 되므로 로봇 좌표나 mission 상태를 넣지 않는다.
        """
        canvas = np.zeros((360, 720, 3), dtype=np.uint8)
        canvas[:] = (18, 22, 28)
        cv2.putText(canvas, "LEGACY CONTROL FEED", (155, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "Use browser UI: /", (205, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (220, 230, 240), 2, cv2.LINE_AA)
        cv2.putText(canvas, "No live pose / no drive command on this feed", (90, 235),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (150, 165, 180), 1, cv2.LINE_AA)
        ok, jpeg = cv2.imencode('.jpg', canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            self._latest_frame = canvas
            self._latest_jpeg = jpeg.tobytes()

    def set_goal(self, x, y):
        with self._lock:
            self.current_goal = {'x': float(x), 'y': float(y), 'time': time.time()}

    def set_nav_status(self, status):
        with self._lock:
            self.nav_status = status
            target = status.get('current_target')
            if target:
                self.current_goal = {'x': target[0], 'y': target[1], 'time': time.time()}
            elif status.get('state') in ('COMPLETED', 'STOPPED', 'IDLE'):
                self.current_goal = None

    def set_map(self, map_yaml_path):
        """R-7 웹 전환: 좌표 프로파일이 바뀌면 화면 지도도 바꾼다. 못 읽으면 합성 지도(표시됨)로 떨어진다."""
        self.map_yaml_path = map_yaml_path
        self._load_map()
        return self.map_source

    def _load_map(self):
        try:
            if os.path.exists(self.map_yaml_path):
                with open(self.map_yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                self.resolution = data.get('resolution', 0.05)
                self.origin = data.get('origin', [-2.325, -2.100, 0.0])
                img_name = data.get('image', 'my_map.pgm')
                img_dir = os.path.dirname(self.map_yaml_path)
                pgm_path = os.path.join(img_dir, img_name)
                
                if os.path.exists(pgm_path):
                    raw_map = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
                    if raw_map is not None:
                        color_map = cv2.cvtColor(raw_map, cv2.COLOR_GRAY2BGR)
                        unexplored = (raw_map > 200) & (raw_map < 210)
                        free_space = (raw_map >= 250)
                        obstacles = (raw_map <= 50)
                        
                        themed = np.zeros_like(color_map)
                        themed[:] = (28, 33, 40) # 배경
                        themed[unexplored] = (40, 48, 58)
                        themed[free_space] = (75, 88, 102) # 주행 가능 영역
                        themed[obstacles] = (0, 220, 255) # 장애물/벽 (골드 하이라이트)

                        self.map_img = themed
                        self.map_h, self.map_w = raw_map.shape[:2]
                        self.map_source = pgm_path
                        print(f"[ControlRenderer] Loaded map: {pgm_path} ({self.map_w}x{self.map_h}, res={self.resolution})")
                        return
        except Exception as e:
            print(f"[ControlRenderer] Map load warning: {e}")

        self._create_fallback_map()

    def _create_fallback_map(self):
        """맵 파일을 못 읽었을 때의 **합성** 그림.

        🔴 예전에는 아무 표시 없이 그렸다. 관제사가 이것을 map4 로 읽고 지휘하면
           좌표가 통째로 허구가 된다(2026-09-18 감사). 그래서 그림에 박고
           `map_source` 로도 내보낸다 — 화면이 스스로 합성임을 말해야 한다.
        """
        w, h = 600, 600
        map_img = np.full((h, w, 3), (30, 35, 45), dtype=np.uint8)
        for x in range(0, w, 40):
            cv2.line(map_img, (x, 0), (x, h), (45, 52, 65), 1)
        for y in range(0, h, 40):
            cv2.line(map_img, (0, y), (w, y), (45, 52, 65), 1)
        cv2.rectangle(map_img, (40, 40), (w - 40, h - 40), (0, 220, 255), 3)
        cv2.rectangle(map_img, (100, 180), (220, 420), (120, 140, 160), -1)
        cv2.rectangle(map_img, (380, 180), (500, 420), (120, 140, 160), -1)
        cv2.rectangle(map_img, (0, 0), (w - 1, 34), (0, 0, 120), -1)
        cv2.putText(map_img, "SYNTHETIC - MAP NOT LOADED", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        self.map_img = map_img
        self.map_w, self.map_h = w, h
        self.map_source = "synthetic"   # /api/status.map_meta 가 이 값을 싣는다
        self.origin = [-2.325, -2.100, 0.0]
        self.resolution = 0.05

    def world_to_pixel(self, wx, wy):
        """월드 좌표(m) -> 맵 픽셀 좌표(px) 변환"""
        px = int((wx - self.origin[0]) / self.resolution)
        py = int(self.map_h - ((wy - self.origin[1]) / self.resolution))
        return px, py

    def update_robot_pose(self, robot_id, x, y, yaw):
        with self._lock:
            if robot_id in self.robots:
                self.robots[robot_id]['x'] = x
                self.robots[robot_id]['y'] = y
                self.robots[robot_id]['yaw'] = yaw
                trail = self.robots[robot_id]['trail']
                trail.append((x, y))
                if len(trail) > 120:
                    trail.pop(0)

    @staticmethod
    def _pose_text(r, with_yaw=True):
        """좌표 문자열. **못 받았으면 숫자를 만들지 않는다.**

        🔴 좌표 기본값을 None 으로 바꾸면서(이 파일 상단) 마커 루프에만 가드를 넣고
           이 텍스트를 빠뜨렸다 — `None` 은 `+.2f` 로 포맷되지 않아 렌더 스레드가
           기동 즉시 죽었다(2026-09-19 적대적 검토가 잡음). 같은 계약 변경의
           **소비자를 전수로 세지 않은** 내 오류다.
        """
        if r.get('x') is None:
            return "Pose: 관측 없음"
        base = "Pose X: %+.2f m  |  Y: %+.2f m" % (r['x'], r['y'])
        if with_yaw and r.get('yaw') is not None:
            base += "  |  Yaw: %.1f deg" % math.degrees(r['yaw'])
        return base

    def _render_loop(self):
        while self.is_running:
            try:
                self._render_once()
            except Exception as exc:
                # 한 프레임의 실패가 스레드를 죽이면 `/control_feed` 가 통째로 멎고
                # 화면은 그 사실을 말하지 않는다. 로그를 남기고 다음 회전으로 간다.
                print("[ControlRenderer] render error: %r" % (exc,))
                time.sleep(0.5)

    def _render_once(self):
        if True:
            canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
            canvas[:] = (18, 22, 28) # 모던 다크 배경

            with self._lock:
                map_canvas = cv2.resize(self.map_img, (640, 640), interpolation=cv2.INTER_NEAREST)
                scale_x = 640.0 / max(self.map_w, 1)
                scale_y = 640.0 / max(self.map_h, 1)

                def to_screen(wx, wy):
                    px = (wx - self.origin[0]) / self.resolution
                    py = self.map_h - ((wy - self.origin[1]) / self.resolution)
                    return int(px * scale_x), int(py * scale_y)

                # 궤적 그리기
                for rid, rdata in self.robots.items():
                    trail = rdata['trail']
                    color = rdata['color']
                    for i in range(1, len(trail)):
                        p1 = to_screen(trail[i-1][0], trail[i-1][1])
                        p2 = to_screen(trail[i][0], trail[i][1])
                        cv2.line(map_canvas, p1, p2, color, 1, cv2.LINE_AA)

                # 경유지 마커 (1, 2, 3)
                for wp in self.waypoints:
                    wpx, wpy = to_screen(wp['x'], wp['y'])
                    if 0 <= wpx < 640 and 0 <= wpy < 640:
                        cv2.circle(map_canvas, (wpx, wpy), 8, wp['color'], -1)
                        cv2.circle(map_canvas, (wpx, wpy), 10, (255, 255, 255), 1)
                        cv2.putText(map_canvas, str(wp['id']), (wpx - 4, wpy + 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

                # 활성 목표 지점 (Target Crosshair)
                if self.current_goal:
                    gx, gy = to_screen(self.current_goal['x'], self.current_goal['y'])
                    if 0 <= gx < 640 and 0 <= gy < 640:
                        pulse = int(11 + 4 * math.sin(time.time() * 8))
                        cv2.circle(map_canvas, (gx, gy), pulse, (0, 255, 255), 2, cv2.LINE_AA)
                        cv2.circle(map_canvas, (gx, gy), 4, (0, 0, 255), -1, cv2.LINE_AA)
                        cv2.line(map_canvas, (gx - 14, gy), (gx + 14, gy), (0, 255, 255), 1, cv2.LINE_AA)
                        cv2.line(map_canvas, (gx, gy - 14), (gx, gy + 14), (0, 255, 255), 1, cv2.LINE_AA)
                        cv2.putText(map_canvas, "GOAL", (gx + 10, gy - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

                # 로봇 마커 그리기
                for rid, rdata in self.robots.items():
                    if rdata['x'] is None:
                        continue          # 관측 없음 — 마커를 찍지 않는다
                    rx, ry = to_screen(rdata['x'], rdata['y'])
                    if 0 <= rx < 640 and 0 <= ry < 640:
                        color = rdata['color']
                        yaw = rdata['yaw']
                        
                        cv2.circle(map_canvas, (rx, ry), 6, color, -1)
                        cv2.circle(map_canvas, (rx, ry), 7, (255, 255, 255), 1)
                        
                        arrow_len = 14
                        ax = int(rx + arrow_len * math.cos(yaw))
                        ay = int(ry - arrow_len * math.sin(yaw))
                        cv2.arrowedLine(map_canvas, (rx, ry), (ax, ay), (255, 255, 255), 1, tipLength=0.35)

                        cv2.putText(map_canvas, rdata['name'], (rx + 8, ry - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            canvas[40:680, 40:680] = map_canvas
            cv2.rectangle(canvas, (40, 40), (680, 680), (70, 85, 105), 2)

            # 2. 우측 관제 현황 패널 (680 ~ 1240)
            cv2.rectangle(canvas, (720, 40), (1240, 680), (25, 30, 40), -1)
            cv2.rectangle(canvas, (720, 40), (1240, 680), (50, 65, 85), 2)

            cv2.putText(canvas, "FIELD CONTROL ROOM MONITOR", (745, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}", (745, 115),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 195, 210), 1, cv2.LINE_AA)
            cv2.line(canvas, (745, 130), (1215, 130), (50, 65, 85), 1)

            # 로봇 1 상태 박스
            r1 = self.robots['robot1']
            nav_state = self.nav_status.get('state', 'IDLE')
            nav_mission = self.nav_status.get('mission', '-')
            rem_dist = self.nav_status.get('remaining_dist', 0.0)

            cv2.rectangle(canvas, (745, 150), (1215, 280), (32, 40, 52), -1)
            cv2.rectangle(canvas, (745, 150), (1215, 280), r1['color'], 2)
            cv2.putText(canvas, f"{r1['name']}", (765, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(canvas, self._pose_text(r1),
                        (765, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 230, 255), 1)
            
            status_color = (0, 255, 150) if nav_state == 'NAVIGATING' else (0, 220, 255)
            cv2.putText(canvas, f"Nav State: {nav_state}  |  Rem: {rem_dist:.2f}m", 
                        (765, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1)
            if nav_mission and nav_mission != '-':
                cv2.putText(canvas, f"Mission: {nav_mission}", 
                            (765, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)

            # 로봇 2 상태 박스
            r2 = self.robots['robot2']
            cv2.rectangle(canvas, (745, 295), (1215, 410), (32, 40, 52), -1)
            cv2.rectangle(canvas, (745, 295), (1215, 410), r2['color'], 2)
            cv2.putText(canvas, f"{r2['name']}", (765, 325), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(canvas, self._pose_text(r2, with_yaw=False), (765, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 230, 200), 1)
            _r2_head = ("Heading: %.1f deg" % math.degrees(r2['yaw'])) if r2.get('yaw') is not None else "Heading: 관측 없음"
            cv2.putText(canvas, f"{_r2_head}  |  Status: STANDBY / PATROL", (765, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)

            # 미션 경로 정보 박스
            cv2.rectangle(canvas, (745, 425), (1215, 650), (32, 40, 52), -1)
            cv2.rectangle(canvas, (745, 425), (1215, 650), (60, 75, 95), 1)
            cv2.putText(canvas, "MISSION WAYPOINTS (READ ONLY)", (765, 455), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            
            y_offset = 490
            for wp in self.waypoints:
                cv2.circle(canvas, (775, y_offset - 5), 8, wp['color'], -1)
                cv2.putText(canvas, f"Point {wp['id']} : {wp['name']} (x={wp['x']:.1f}, y={wp['y']:.1f})", 
                            (795, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 220, 230), 1)
                y_offset += 32

            cv2.putText(canvas, "Mission 1: 1 -> 2 -> 1 (Avoid Shelf 1 via South)", (765, 595),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1)
            cv2.putText(canvas, "Mission 2: 1 -> 2 -> 3 -> 1 (Complete Factory Tour)", (765, 625),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (100, 255, 100), 1)

            # JPEG 인코딩
            _, jpeg = cv2.imencode('.jpg', canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            jpeg_bytes = jpeg.tobytes()

            with self._lock:
                self._latest_frame = canvas
                self._latest_jpeg = jpeg_bytes

            time.sleep(0.1) # 10 FPS

    def get_latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg

    def get_robot_data(self):
        with self._lock:
            data = {}
            for rid, r in self.robots.items():
                # ⭐ 좌표가 None 이면 **키를 아예 안 싣는다**. 소비자가 `.get('x', 0)`
                #    으로 0 을 만들어 쓰는 길을 남기지 않는다.
                item = {
                    'name': r['name'],
                    'observed': r['x'] is not None,
                    'color': f"rgb({r['color'][2]},{r['color'][1]},{r['color'][0]})"
                }
                if r['x'] is not None:
                    item['x'] = r['x']
                    item['y'] = r['y']
                    item['yaw'] = r['yaw']
                data[rid] = item
            return data

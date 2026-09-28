#!/usr/bin/env python3
"""
Relay Station Native Mission Controller GUI (Tkinter)
- 현장 중계 노트북(로컬) 전용 로봇 제어 콘솔
- 2D SLAM 맵 기반 Click & Drag 마우스 목표 설정
- 실시간 실물 로봇(Pinky #1) & Gazebo 시뮬레이션 위치 동기화 렌더링
- 프리셋 미션 1, 2 및 빠른 경유지 원클릭 기동
- 대형 비상 정지(EMERGENCY STOP) 버튼
- 실측 오차(Δ 거리, Δ 각도) 실시간 HUD
"""

import sys
import os
import time
import math
import json
import threading
import urllib.request
import urllib.error

import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

# Auto-detect display if running under background daemon
if 'DISPLAY' not in os.environ or not os.environ['DISPLAY']:
    os.environ['DISPLAY'] = ':0'
if 'XAUTHORITY' not in os.environ or not os.environ['XAUTHORITY']:
    import glob
    auths = glob.glob('/run/user/1000/.mutter-Xwaylandauth.*')
    if auths:
        os.environ['XAUTHORITY'] = auths[0]

SERVER_URL = "http://127.0.0.1:8889"
MAP_PNG_PATH = "$HOME/field_gateway_relay/static/my_map.png"
if not os.path.exists(MAP_PNG_PATH):
    MAP_PNG_PATH = "$HOME/my_map.png"

# 맵 메타데이터 (my_map.yaml 기준)
MAP_ORIGIN_X = -2.325
MAP_ORIGIN_Y = -2.100
MAP_RES = 0.05
MAP_W_CELLS = 93
MAP_H_CELLS = 84

CANVAS_W = 520
CANVAS_H = 470


class RelayMissionControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🛰️ 현장 중계 장비 로봇 제어 콘솔 [Mission Control Station]")
        self.root.geometry("980x620")
        self.root.configure(bg="#0d1117")

        # Telemetry State
        self.real_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.sim_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.active_goal = None
        self.discrepancy = {'dist_err_cm': 0.0, 'dyaw_deg': 0.0}
        self.nav_state = "IDLE"

        # Drag State for Goal Heading
        self.is_dragging = False
        self.drag_start_world = None
        self.drag_current_canvas = None

        self._init_ui()
        self._load_map()

        # Background polling worker
        self.running = True
        self.poll_thread = threading.Thread(target=self._poll_status_loop, daemon=True)
        self.poll_thread.start()

    def _init_ui(self):
        # 1. Top Header
        top_frame = tk.Frame(self.root, bg="#161b22", height=45, bd=1, relief="solid")
        top_frame.pack(side=tk.TOP, fill=tk.X)

        title_lbl = tk.Label(top_frame, text="🛰️ Pinky Multi-Robot Mission Controller (중계 장비 전용)",
                             font=("Pretendard", 12, "bold"), fg="#58a6ff", bg="#161b22")
        title_lbl.pack(side=tk.LEFT, padx=16, pady=8)

        self.sec_badge = tk.Label(top_frame, text="🛡️ LOCALHOST SECURED",
                                  font=("Consolas", 9, "bold"), fg="#3fb950", bg="#238636", padx=8, pady=2)
        self.sec_badge.pack(side=tk.RIGHT, padx=16, pady=8)

        # Main Split Frame
        main_frame = tk.Frame(self.root, bg="#0d1117")
        main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=12, pady=10)

        # 2. Left: 2D SLAM Canvas
        map_card = tk.Frame(main_frame, bg="#161b22", bd=1, relief="solid")
        map_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        map_hdr = tk.Frame(map_card, bg="#21262d", height=30)
        map_hdr.pack(side=tk.TOP, fill=tk.X)
        tk.Label(map_hdr, text="🗺️ 2D SLAM 지도 (마우스 클릭 & 드래그로 목표 지정)",
                 font=("Pretendard", 10, "bold"), fg="#f0f6fc", bg="#21262d").pack(side=tk.LEFT, padx=10, pady=4)
        self.coord_hover_lbl = tk.Label(map_hdr, text="X: 0.00m, Y: 0.00m",
                                        font=("Consolas", 9), fg="#7ee787", bg="#21262d")
        self.coord_hover_lbl.pack(side=tk.RIGHT, padx=10, pady=4)

        self.canvas = tk.Canvas(map_card, width=CANVAS_W, height=CANVAS_H, bg="#0f141c", highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.canvas.bind("<ButtonPress-1>", self._on_canvas_down)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_up)
        self.canvas.bind("<Motion>", self._on_canvas_hover)

        # Map footer guide
        tk.Label(map_card, text="💡 주황색 = 실물 로봇(#1), 하늘색 = 가제보 시뮬레이션, 빨간핀 = 목표 지점",
                 font=("Pretendard", 8), fg="#8b949e", bg="#161b22").pack(side=tk.BOTTOM, pady=4)

        # 3. Right: Control & Mission Panel
        ctrl_card = tk.Frame(main_frame, bg="#161b22", width=380, bd=1, relief="solid")
        ctrl_card.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(0, 0))
        ctrl_card.pack_propagate(False)

        # Status & Discrepancy Box
        status_box = tk.LabelFrame(ctrl_card, text="📊 실시간 실측 오차 HUD", bg="#161b22", fg="#ffcc00",
                                   font=("Pretendard", 9, "bold"), padx=10, pady=8)
        status_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(10, 6))

        self.lbl_dist_err = tk.Label(status_box, text="거리 편차 (ΔDist): 0.0 cm",
                                     font=("Consolas", 11, "bold"), fg="#3fb950", bg="#161b22")
        self.lbl_dist_err.pack(anchor=tk.W)

        self.lbl_yaw_err = tk.Label(status_box, text="각도 편차 (ΔYaw) : 0.0°",
                                    font=("Consolas", 9), fg="#8b949e", bg="#161b22")
        self.lbl_yaw_err.pack(anchor=tk.W, pady=(2, 0))

        self.lbl_nav_state = tk.Label(status_box, text="주행 상태: IDLE (대기중)",
                                      font=("Pretendard", 9, "bold"), fg="#58a6ff", bg="#161b22")
        self.lbl_nav_state.pack(anchor=tk.W, pady=(4, 0))

        # Coordinate Manual Input
        input_box = tk.LabelFrame(ctrl_card, text="🎯 좌표 직접 입력 목표 전송", bg="#161b22", fg="#58a6ff",
                                  font=("Pretendard", 9, "bold"), padx=10, pady=8)
        input_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=6)

        row1 = tk.Frame(input_box, bg="#161b22")
        row1.pack(fill=tk.X, pady=2)
        tk.Label(row1, text="X (m):", fg="#c9d1d9", bg="#161b22", width=6).pack(side=tk.LEFT)
        self.ent_x = tk.Entry(row1, width=8, bg="#0d1117", fg="#fff", insertbackground="#fff")
        self.ent_x.insert(0, "0.0")
        self.ent_x.pack(side=tk.LEFT, padx=4)

        tk.Label(row1, text="Y (m):", fg="#c9d1d9", bg="#161b22", width=6).pack(side=tk.LEFT, padx=(6, 0))
        self.ent_y = tk.Entry(row1, width=8, bg="#0d1117", fg="#fff", insertbackground="#fff")
        self.ent_y.insert(0, "0.0")
        self.ent_y.pack(side=tk.LEFT, padx=4)

        row2 = tk.Frame(input_box, bg="#161b22")
        row2.pack(fill=tk.X, pady=4)
        tk.Label(row2, text="Yaw(°):", fg="#c9d1d9", bg="#161b22", width=6).pack(side=tk.LEFT)
        self.ent_yaw = tk.Entry(row2, width=8, bg="#0d1117", fg="#fff", insertbackground="#fff")
        self.ent_yaw.insert(0, "0.0")
        self.ent_yaw.pack(side=tk.LEFT, padx=4)

        btn_send = tk.Button(row2, text="🚀 목표 전송", bg="#238636", fg="#fff", font=("Pretendard", 9, "bold"),
                             command=self._on_manual_send, activebackground="#2ea043")
        btn_send.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=4)

        # Quick Waypoints
        wp_box = tk.LabelFrame(ctrl_card, text="📍 지정 작업대 바로가기", bg="#161b22", fg="#c9d1d9",
                               font=("Pretendard", 9, "bold"), padx=10, pady=8)
        wp_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=6)

        wp_row = tk.Frame(wp_box, bg="#161b22")
        wp_row.pack(fill=tk.X)
        tk.Button(wp_row, text="📍 Point 1 (원점)", bg="#21262d", fg="#ff9900", font=("Pretendard", 8),
                  command=lambda: self._send_goal(0.0, 0.0, 0.0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Button(wp_row, text="📍 Point 2 (좌측)", bg="#21262d", fg="#3fb950", font=("Pretendard", 8),
                  command=lambda: self._send_goal(-1.4, 0.0, 0.0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Button(wp_row, text="📍 Point 3 (우측)", bg="#21262d", fg="#a371f7", font=("Pretendard", 8),
                  command=lambda: self._send_goal(1.3, -0.3, 0.0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Missions
        mission_box = tk.LabelFrame(ctrl_card, text="🚩 프리셋 자율주행 시나리오", bg="#161b22", fg="#c9d1d9",
                                    font=("Pretendard", 9, "bold"), padx=10, pady=8)
        mission_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=6)

        tk.Button(mission_box, text="🚩 미션 1 (1 → 2 → 1 선반 회피 왕복)", bg="#1f6feb", fg="#fff",
                  font=("Pretendard", 9, "bold"), command=lambda: self._send_mission(1)).pack(fill=tk.X, pady=3)
        tk.Button(mission_box, text="🚩 미션 2 (1 → 2 → 3 → 1 전 구역 순회)", bg="#8957e5", fg="#fff",
                  font=("Pretendard", 9, "bold"), command=lambda: self._send_mission(2)).pack(fill=tk.X, pady=3)

        # Emergency Stop Button
        btn_stop = tk.Button(ctrl_card, text="🛑 비상 정지 (EMERGENCY STOP)", bg="#da3633", fg="#ffffff",
                             font=("Pretendard", 12, "bold"), height=2, activebackground="#f85149",
                             command=self._send_stop)
        btn_stop.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=12)

    def _load_map(self):
        try:
            if os.path.exists(MAP_PNG_PATH):
                pil_img = Image.open(MAP_PNG_PATH)
                pil_img = pil_img.resize((CANVAS_W, CANVAS_H), Image.Resampling.NEAREST)
                self.map_img_tk = ImageTk.PhotoImage(pil_img)
            else:
                self.map_img_tk = None
        except Exception as e:
            print("[GUI] Error loading map image:", e)
            self.map_img_tk = None

    def _world_to_canvas(self, wx, wy):
        px = (wx - MAP_ORIGIN_X) / (MAP_W_CELLS * MAP_RES) * CANVAS_W
        py = (1.0 - (wy - MAP_ORIGIN_Y) / (MAP_H_CELLS * MAP_RES)) * CANVAS_H
        return px, py

    def _canvas_to_world(self, cx, cy):
        wx = MAP_ORIGIN_X + (cx / CANVAS_W) * (MAP_W_CELLS * MAP_RES)
        wy = MAP_ORIGIN_Y + (1.0 - (cy / CANVAS_H)) * (MAP_H_CELLS * MAP_RES)
        return round(wx, 3), round(wy, 3)

    def _on_canvas_hover(self, event):
        wx, wy = self._canvas_to_world(event.x, event.y)
        self.coord_hover_lbl.config(text=f"X: {wx:+.2f}m, Y: {wy:+.2f}m")

    def _on_canvas_down(self, event):
        self.is_dragging = True
        wx, wy = self._canvas_to_world(event.x, event.y)
        self.drag_start_world = (wx, wy)
        self.drag_current_canvas = (event.x, event.y)

    def _on_canvas_drag(self, event):
        if self.is_dragging:
            self.drag_current_canvas = (event.x, event.y)
            self._render()

    def _on_canvas_up(self, event):
        if not self.is_dragging or not self.drag_start_world:
            self.is_dragging = False
            return

        gx, gy = self.drag_start_world
        end_wx, end_wy = self._canvas_to_world(event.x, event.y)
        dx = end_wx - gx
        dy = end_wy - gy

        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.08 else self.real_pose['yaw']
        self.is_dragging = False
        self.drag_start_world = None
        self.drag_current_canvas = None

        self._send_goal(gx, gy, yaw)

    def _on_manual_send(self):
        try:
            x = float(self.ent_x.get().strip())
            y = float(self.ent_y.get().strip())
            yaw_deg = float(self.ent_yaw.get().strip())
            yaw = math.radians(yaw_deg)
            self._send_goal(x, y, yaw)
        except ValueError:
            messagebox.showerror("입력 오류", "유효한 숫자를 입력해주세요.")

    def _send_goal(self, x, y, yaw=0.0):
        self.active_goal = {'x': x, 'y': y, 'yaw': yaw}
        self.ent_x.delete(0, tk.END); self.ent_x.insert(0, f"{x:.2f}")
        self.ent_y.delete(0, tk.END); self.ent_y.insert(0, f"{y:.2f}")
        self.ent_yaw.delete(0, tk.END); self.ent_yaw.insert(0, f"{math.degrees(yaw):.1f}")

        def req():
            # 2026-09-26 (관제 검수 S1): 게이트웨이가 ESTOP·정지·HOLD·완료 중 목표를 409 로 거절한다 —
            # 거절된 목표를 화면에 '가는 중' 으로 남기지 않는다.
            try:
                data = json.dumps({'x': x, 'y': y, 'yaw': yaw}).encode('utf-8')
                req = urllib.request.Request(f"{SERVER_URL}/api/robot1/goal", data=data,
                                             headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=2.0) as res:
                    pass
            except urllib.error.HTTPError as e:
                try:
                    body = json.loads(e.read().decode('utf-8') or '{}')
                except Exception:
                    body = {}
                self.active_goal = None
                print("[GUI] 목표 거절(%s): %s" % (e.code, body.get('message') or body.get('reason')))
            except Exception as e:
                self.active_goal = None
                print("[GUI] Error sending goal:", e)
            finally:
                try:
                    self.root.after(0, self._render)
                except Exception:
                    pass
        threading.Thread(target=req, daemon=True).start()
        self._render()

    def _send_mission(self, mission_num):
        def req():
            try:
                data = json.dumps({'mission': str(mission_num)}).encode('utf-8')
                req = urllib.request.Request(f"{SERVER_URL}/api/robot1/mission", data=data,
                                             headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=2.0) as res:
                    body = json.loads(res.read().decode('utf-8') or '{}')
                # 202 = 보냈지만 받는 쪽은 모른다 — 성공이라 하지 않는다
                print("[GUI] 미션 %s:" % ('보냄' if body.get('dispatched') else '응답'), body.get('message'))
            except urllib.error.HTTPError as e:
                try:
                    body = json.loads(e.read().decode('utf-8') or '{}')
                except Exception:
                    body = {}
                print("[GUI] 미션 거절(%s): %s" % (e.code, body.get('message') or body.get('reason')))
            except Exception as e:
                print("[GUI] Error sending mission:", e)
        threading.Thread(target=req, daemon=True).start()

    def _send_stop(self):
        """🔴 응답을 **읽는다**. 그리고 목표는 성공을 확인한 뒤에 지운다.

        예전에는 요청을 보내기도 전에 `active_goal = None` 을 했고 응답도 버렸다 —
        망이 죽어 정지가 안 갔는데 화면에서는 목표가 사라졌다. 화면이 '멈췄다' 고
        말하는 동안 로봇은 그대로다.
        """
        def req():
            try:
                rq = urllib.request.Request(f"{SERVER_URL}/api/robot1/stop", data=b'{}',
                                            headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(rq, timeout=2.0) as res:
                    body = json.loads(res.read().decode('utf-8') or '{}')
                # 2026-09-25: success = 로봇이 STOP 을 처리하고 멈췄다고 **보고**했다(200). dispatched = 보내기만
                # 했다(202 — 10 Hz 로 계속 보낸다). 예전 `subscribers` 는 D7 에서 없어졌다.
                if body.get('success'):
                    self.active_goal = None
                    print("[GUI] 정지 확인:", body.get('message'))
                elif body.get('dispatched'):
                    self.active_goal = None
                    print("[GUI] 정지 보냄 — 확인 전:", body.get('message') or body.get('reason'))
                else:
                    print("[GUI] 정지 불가:", body.get('message') or body.get('reason'))
            except urllib.error.HTTPError as e:
                try:
                    body = json.loads(e.read().decode('utf-8') or '{}')
                except Exception:
                    body = {}
                print("[GUI] 정지 불가(%s): %s" % (e.code, body.get('message') or body.get('reason')))
            except Exception as e:
                print("[GUI] 정지 전달 실패:", e)
            finally:
                try:
                    self.root.after(0, self._render)
                except Exception:
                    pass
        threading.Thread(target=req, daemon=True).start()

    def _poll_status_loop(self):
        while self.running:
            try:
                with urllib.request.urlopen(f"{SERVER_URL}/api/status", timeout=1.0) as res:
                    data = json.loads(res.read().decode('utf-8'))
                    if 'robots' in data:
                        # 🔴 좌표는 **관측됐을 때만** 실린다 — 미관측이면 키가 없다.
                        #    첨자로 꺼내면 KeyError 가 나고 아래 `except: pass` 가
                        #    삼켜서 HUD 가 마지막 값을 든 채 멈춘다(멈춘 줄 모른다).
                        r = data['robots'].get('robot1') or {}
                        self.real_pose = ({'x': r['x'], 'y': r['y'], 'yaw': r['yaw']}
                                          if r.get('observed') and r.get('x') is not None else None)
                        g = data['robots'].get('gazebo_sim') or {}
                        self.sim_pose = ({'x': g['x'], 'y': g['y'], 'yaw': g['yaw']}
                                         if g.get('observed') and g.get('x') is not None else None)
                    if 'discrepancy' in data:
                        self.discrepancy = data['discrepancy']
                    if 'robot1_nav' in data and data['robot1_nav']:
                        self.nav_state = data['robot1_nav'].get('state', 'IDLE')

                self.root.after(0, self._update_hud)
            except Exception as exc:
                # 조용히 삼키면 HUD 가 낡은 값을 든 채 멈춘다. 최소한 말은 한다.
                print("[GUI] status poll 실패:", exc)
            time.sleep(0.1)

    def _update_hud(self):
        dist_cm = self.discrepancy.get('dist_err_cm', 0.0)
        dyaw = self.discrepancy.get('dyaw_deg', 0.0)
        self.lbl_dist_err.config(text=f"거리 편차 (ΔDist): {dist_cm:.1f} cm",
                                 fg="#3fb950" if dist_cm < 10.0 else ("#f2cc60" if dist_cm < 25.0 else "#f85149"))
        self.lbl_yaw_err.config(text=f"각도 편차 (ΔYaw) : {dyaw:+.1f}°")
        self.lbl_nav_state.config(text=f"주행 상태: {self.nav_state}")
        self._render()

    def _render(self):
        self.canvas.delete("all")

        # 1. Base Map
        if self.map_img_tk:
            self.canvas.create_image(0, 0, image=self.map_img_tk, anchor=tk.NW)

        # 2. Grid lines (1m)
        for mx in range(-2, 3):
            p1x, _ = self._world_to_canvas(mx, -2.0)
            self.canvas.create_line(p1x, 0, p1x, CANVAS_H, fill="#1f2937", dash=(2, 2))
        for my in range(-2, 3):
            _, p1y = self._world_to_canvas(0, my)
            self.canvas.create_line(0, p1y, CANVAS_W, p1y, fill="#1f2937", dash=(2, 2))

        # 3. Gazebo Sim Robot (Cyan)
        gx, gy = self._world_to_canvas(self.sim_pose['x'], self.sim_pose['y'])
        self.canvas.create_oval(gx - 7, gy - 7, gx + 7, gy + 7, fill="#00ffff", outline="#ffffff", width=1)
        ax = gx + 16 * math.cos(self.sim_pose['yaw'])
        ay = gy - 16 * math.sin(self.sim_pose['yaw'])
        self.canvas.create_line(gx, gy, ax, ay, fill="#00ffff", width=2)

        # 4. Real Robot 1 (Orange)
        rx, ry = self._world_to_canvas(self.real_pose['x'], self.real_pose['y'])
        self.canvas.create_oval(rx - 8, ry - 8, rx + 8, ry + 8, fill="#ff9900", outline="#ffffff", width=2)
        rax = rx + 18 * math.cos(self.real_pose['yaw'])
        ray = ry - 18 * math.sin(self.real_pose['yaw'])
        self.canvas.create_line(rx, ry, rax, ray, fill="#ffffff", width=2)
        self.canvas.create_text(rx, ry + 16, text="Pinky #1", fill="#ff9900", font=("Consolas", 8, "bold"))

        # 5. Active Goal (Yellow Target)
        if self.active_goal:
            tgx, tgy = self._world_to_canvas(self.active_goal['x'], self.active_goal['y'])
            self.canvas.create_oval(tgx - 9, tgy - 9, tgx + 9, tgy + 9, outline="#ffcc00", width=2)
            self.canvas.create_line(tgx - 12, tgy, tgx + 12, tgy, fill="#ffcc00", width=1)
            self.canvas.create_line(tgx, tgy - 12, tgx, tgy + 12, fill="#ffcc00", width=1)
            if 'yaw' in self.active_goal:
                tax = tgx + 20 * math.cos(self.active_goal['yaw'])
                tay = tgy - 20 * math.sin(self.active_goal['yaw'])
                self.canvas.create_line(tgx, tgy, tax, tay, fill="#ffcc00", width=2, arrow=tk.LAST)
            self.canvas.create_text(tgx, tgy - 16, text="GOAL", fill="#ffcc00", font=("Consolas", 8, "bold"))

        # 6. Dragging Arrow Preview
        if self.is_dragging and self.drag_start_world and self.drag_current_canvas:
            sx, sy = self._world_to_canvas(self.drag_start_world[0], self.drag_start_world[1])
            ex, ey = self.drag_current_canvas
            self.canvas.create_line(sx, sy, ex, ey, fill="#ff7b72", width=2, arrow=tk.LAST)
            self.canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill="#ff7b72", outline="")


def main():
    root = tk.Tk()
    app = RelayMissionControlApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()

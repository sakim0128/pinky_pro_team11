#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Field Test Runner for Mixed Fleet Station (중계장비 현장 실측 및 순차 실행 도구)
=============================================================================
중계장비(Relay Station)에서 실물 로봇(Pinky1, Pinky2)의 상태를 단계별로 검증하고,
초기화 -> 주행 시작 -> 교차로 양보/통과 실시간 모니터링 -> (선택) 정지/재개 시험 ->
도착/미션 완료 판정까지 순차적으로 수행하며 실시간 로그와 최종 영수증(Receipt)을 생성합니다.

사용법:
  # 대화형 순차 실행 (단계마다 확인 후 진행 - 현장 테스트 추천)
  python3 field_test_runner.py

  # 완전 자동 실행 (시나리오 자동 진행 + 정지/재개 시험 포함)
  python3 field_test_runner.py --auto --test-stop

  # 특정 게이트웨이 주소 지정
  python3 field_test_runner.py --gateway http://127.0.0.1:8889

  # 태블릿 미연결 시 가상 구역 이벤트 자동 주입
  python3 field_test_runner.py --inject-zone-events
"""

import sys
import os
import json
import time
import math
import argparse
from datetime import datetime
from pathlib import Path
import urllib.request
import urllib.error

# 콘솔 UTF-8 및 ANSI 색상 설정
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"

def supports_color():
    if os.environ.get("NO_COLOR"):
        return False
    return sys.stdout.isatty() or os.environ.get("TERM") in ("xterm", "xterm-256color")

USE_COLOR = supports_color()

def c(text, color):
    return f"{color}{text}{Colors.RESET}" if USE_COLOR else str(text)

class FieldLogger:
    def __init__(self, log_file=None):
        self.log_file = log_file
        if log_file:
            Path(log_file).parent.mkdir(parents=True, exist_ok=True)
            self.fh = open(log_file, "a", encoding="utf-8")
        else:
            self.fh = None

    def _write(self, msg):
        if self.fh:
            # ANSI 제거 후 파일 저장
            import re
            plain = re.sub(r'\x1b\[[0-9;]*m', '', msg)
            self.fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {plain}\n")
            self.fh.flush()

    def log(self, tag, msg, color=Colors.WHITE):
        formatted = f"{c(f'[{tag:<6}]', color)} {msg}"
        print(formatted)
        self._write(f"[{tag:<6}] {msg}")

    def info(self, msg): self.log("INFO", msg, Colors.CYAN)
    def ok(self, msg): self.log("PASS", msg, Colors.GREEN)
    def warn(self, msg): self.log("WARN", msg, Colors.YELLOW)
    def err(self, msg): self.log("FAIL", msg, Colors.RED)
    def step(self, num, title):
        banner = "=" * 65
        print(f"\n{c(banner, Colors.BLUE)}")
        print(c(f"  STEP {num}: {title}", Colors.BOLD + Colors.BLUE))
        print(f"{c(banner, Colors.BLUE)}\n")
        self._write(f"=== STEP {num}: {title} ===")

    def milestone(self, event_name, detail):
        t_str = datetime.now().strftime('%H:%M:%S.%f')[:-4]
        msg = f"{c(f'[{t_str}] 🌟 {event_name}:', Colors.BOLD + Colors.MAGENTA)} {detail}"
        print(f"\n{msg}")
        self._write(f"[MILESTONE] {event_name}: {detail}")

    def close(self):
        if self.fh:
            self.fh.close()


class MixedFleetGatewayClient:
    def __init__(self, base_url, logger):
        self.base = base_url.rstrip('/')
        self.log = logger

    def get(self, path, timeout=3.0):
        url = self.base + path
        req = urllib.request.Request(url, headers={'User-Agent': 'FieldTestRunner/1.0'})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.status, json.load(resp)
        except urllib.error.HTTPError as e:
            try: body = json.loads(e.read().decode('utf-8'))
            except Exception: body = {'error': str(e)}
            return e.code, body
        except Exception as e:
            return 0, {'error': str(e)}

    def post(self, path, payload=None, key=None, timeout=4.0):
        url = self.base + path
        data = json.dumps(payload or {}).encode('utf-8')
        headers = {'Content-Type': 'application/json', 'User-Agent': 'FieldTestRunner/1.0'}
        if key:
            headers['X-API-Key'] = key
        req = urllib.request.Request(url, data=data, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.status, json.load(resp)
        except urllib.error.HTTPError as e:
            try: body = json.loads(e.read().decode('utf-8'))
            except Exception: body = {'error': str(e)}
            return e.code, body
        except Exception as e:
            return 0, {'error': str(e)}


class FieldTestRunner:
    def __init__(self, args):
        self.args = args
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = Path(args.out_dir) / f"run_{ts}"
        self.out_dir.mkdir(parents=True, exist_ok=True)
        
        self.logger = FieldLogger(self.out_dir / "runner.log")
        self.gw = MixedFleetGatewayClient(args.gateway, self.logger)
        
        self.results = {}
        self.telemetry_samples = []
        self.milestones_detected = set()
        
        # 키 감지 (환경변수 또는 로컬 설정)
        self.vision_key = os.environ.get('RELAY_VISION_API_KEY') or os.environ.get('TWIN_API_KEY') or 'FIELD_INJECTOR_KEY'

    def prompt(self, step_title, default=True):
        if self.args.auto:
            self.logger.info(f"[AUTO] {step_title} 자동 진행")
            return True
        prompt_char = "[Y/n]" if default else "[y/N]"
        ans = input(f"\n{c('▶', Colors.BOLD + Colors.GREEN)} {step_title} {prompt_char} (q: 중단): ").strip().lower()
        if ans == 'q':
            self.logger.warn("사용자에 의해 테스트가 중단되었습니다.")
            sys.exit(0)
        return ans in ('y', 'yes', '') if default else ans in ('y', 'yes')

    # =========================================================================
    # Step 0: 사전 환경 및 통신 상태 진단 (Preflight Check)
    # =========================================================================
    def step0_preflight(self):
        self.logger.step(0, "중계 게이트웨이 및 네트워크 헬스체크 (Preflight)")
        self.logger.info(f"게이트웨이 URL: {self.args.gateway}")

        code, data = self.gw.get('/api/status')
        if code != 200:
            self.logger.err(f"게이트웨이 접속 실패 (HTTP {code}): {data.get('error', 'Unknown')}")
            return False

        self.logger.ok("게이트웨이 HTTP API 정상 연결 확인")
        
        # 리소스 및 네트워크
        resources = data.get('host_resources', {})
        if resources:
            self.logger.info(f"중계기 리소스: CPU {resources.get('cpu_percent')}% | Temp {resources.get('cpu_temp')}°C | RAM {resources.get('memory_used_gb')}/{resources.get('memory_total_gb')}GB")

        # 안전 가드 (Twin Guard / Latch) 점검
        guard = data.get('twin_runtime') or {}
        if guard.get('latched'):
            self.logger.warn(f"안전 가드 래치 감지됨: reason={guard.get('reason')}. 자동 래치 해제(Resume) 시도...")
            rcode, rdata = self.gw.post('/api/fleet/resume')
            if rcode == 200 and rdata.get('success'):
                self.logger.ok("안전 가드 래치 해제 성공 (MANUAL_RESUME)")
            else:
                self.logger.err(f"안전 가드 해제 실패 (HTTP {rcode}): {rdata}")
                return False

        self.results['P0_preflight'] = 'PASS'
        return True

    # =========================================================================
    # Step 1: 로봇 검출 및 구성 프로파일 검증 (P0, P1)
    # =========================================================================
    def step1_robot_discovery(self):
        self.logger.step(1, "로봇 하드웨어 인식 및 프로파일 대조 (P0, P1)")

        code, fleet_status = self.gw.get('/api/fleet/status')
        if code != 200:
            self.logger.err(f"플릿 상태 조회 실패 (HTTP {code}): {fleet_status}")
            return False

        profile_raw = fleet_status.get('profile', '')
        profile_name = profile_raw.get('active', '') if isinstance(profile_raw, dict) else str(profile_raw)
        self.logger.info(f"현재 지도 프로파일: {c(profile_name, Colors.BOLD)}")
        if self.args.expected_profile and self.args.expected_profile not in (profile_name, profile_raw.get('default', '') if isinstance(profile_raw, dict) else ''):
            self.logger.warn(f"기대 프로파일({self.args.expected_profile})과 다릅니다. (현재: {profile_name})")

        robots = fleet_status.get('robots', {})
        p1_state = robots.get('pinky1', {})
        p2_state = robots.get('pinky2', {})

        # P0: 구성 확인
        if 'pinky1' in robots and 'pinky2' in robots:
            self.logger.ok(f"로봇 구성 확인: pinky1({p1_state.get('drive_mode')}) · pinky2({p2_state.get('drive_mode')})")
            self.results['P0_config'] = 'PASS'
        else:
            self.logger.err(f"로봇 감지 누락! 감지된 로봇: {list(robots.keys())}")
            self.results['P0_config'] = 'FAIL'
            return False

        # P1: 좌표/하트비트 신선도 확인
        p1_heard = p1_state.get('heard', 999.0)
        p2_heard = p2_state.get('heard', 999.0)
        self.logger.info(f"pinky1 응답 지연: {p1_heard:.3f}s | 위치: ({p1_state.get('x', 0):.2f}, {p1_state.get('y', 0):.2f})")
        self.logger.info(f"pinky2 응답 지연: {p2_heard:.3f}s | 위치: ({p2_state.get('x', 0):.2f}, {p2_state.get('y', 0):.2f})")

        if p1_heard < 1.0 and p2_heard < 1.0:
            self.logger.ok("두 로봇 모두 1초 이내 신선한 하트비트/위치 정보 확인 (P1 PASS)")
            self.results['P1_location'] = 'PASS'
        else:
            self.logger.warn("로봇 통신 지연이 1초를 초과합니다. Wi-Fi 또는 에이전트 데몬을 확인하세요.")
            self.results['P1_location'] = 'WARN'

        return True

    # =========================================================================
    # Step 2: 출발 노드 초기 위치 배정 (Initial Poses)
    # =========================================================================
    def step2_initial_poses(self):
        self.logger.step(2, "출발 노드 초기 위치 배정 (/api/fleet/initial_poses)")
        self.logger.info("로봇 2대가 실제 출발 위치(pinky1: BL, pinky2: BR)에 거치되어 있는지 확인하세요.")

        # 플릿이 STOPPED 상태인 경우 초기 위치 명령이 거부되므로 ASSIGNED 상태로 복귀
        _, fs = self.gw.get('/api/fleet/status')
        if fs.get('mission') == 'STOPPED' or fs.get('mission_state') == 'STOPPED':
            self.logger.info("플릿이 STOPPED 상태입니다. 초기 위치 배정을 위해 상태를 재개(Resume)합니다.")
            self.gw.post('/api/fleet/resume')
            time.sleep(1.0)

        if not self.prompt("초기 위치 배정 명령을 전송하시겠습니까?"):
            return False

        code, body = self.gw.post('/api/fleet/initial_poses')
        if code == 200 and body.get('success'):
            self.logger.ok(f"초기 위치 명령 수락: {body.get('message', 'OK')}")
            time.sleep(2.0)
            return True
        else:
            self.logger.err(f"초기 위치 전송 실패 (HTTP {code}): {body.get('reason') or body.get('message')}")
            return False

    # =========================================================================
    # Step 3: 주행 시작 및 RUNNING 전이 (S1)
    # =========================================================================
    def step3_start_mission(self):
        self.logger.step(3, "미션 주행 시작 트리거 (S1 시작)")

        if not self.prompt("미션 주행을 시작하시겠습니까? (로봇 바퀴 구동 주의)"):
            return False

        code, body = self.gw.post('/api/fleet/start')
        if code != 200 or not body.get('success'):
            self.logger.err(f"주행 시작 명령 거부 (HTTP {code}): {body}")
            self.results['S1_start'] = 'FAIL'
            return False

        self.logger.ok(f"START 명령 전송 완료: {body.get('message', 'START')}")
        
        # RUNNING 전이 대기 (최대 10초)
        start_wait = time.time()
        running = False
        while time.time() - start_wait < 10.0:
            code, fs = self.gw.get('/api/fleet/status')
            if code == 200 and fs.get('mission') == 'RUNNING':
                running = True
                break
            time.sleep(0.3)

        if running:
            self.logger.ok("미션 상태 RUNNING 진입 확인 (S1 PASS)")
            self.results['S1_start'] = 'PASS'
            return True
        else:
            self.logger.err("START 명령 후 미션이 RUNNING 상태로 전이되지 않았습니다.")
            self.results['S1_start'] = 'FAIL'
            return False

    # =========================================================================
    # Step 4: 실시간 주행 및 교차로 모니터링 (S2 ~ S5 & X1, S6, S7)
    # =========================================================================
    def step4_monitor_loop(self):
        self.logger.step(4, "실시간 폐루프 주행 텔레메트리 모니터링 (S2 ~ S7)")
        self.logger.info("실시간 로봇 거동, 속도, 교차로 예약 상태를 모니터링합니다. (Ctrl+C로 중단 가능)")

        t0 = time.time()
        end_time = t0 + self.args.observe_seconds
        
        p1_x0, p1_y0 = None, None
        p1_max_disp = 0.0
        
        stop_tested = not self.args.test_stop
        zone_sent = {}
        
        last_print = 0
        
        try:
            while time.time() < end_time:
                now = time.time()
                elapsed = now - t0

                code, fs = self.gw.get('/api/fleet/status')
                if code != 200:
                    time.sleep(0.3)
                    continue

                self.telemetry_samples.append(dict(fs, _t=elapsed))
                mission = fs.get('mission', 'UNKNOWN')
                holders = fs.get('node_holders', {})
                r = fs.get('robots', {})
                p1 = r.get('pinky1', {})
                p2 = r.get('pinky2', {})

                # 위치 변위 계산
                if p1.get('x') is not None and p1.get('y') is not None:
                    if p1_x0 is None:
                        p1_x0, p1_y0 = p1['x'], p1['y']
                    disp = math.hypot(p1['x'] - p1_x0, p1['y'] - p1_y0)
                    if disp > p1_max_disp:
                        p1_max_disp = disp

                # ── 마일스톤 이벤트 감지 ──
                # S2: 0.3m 이상 이동
                if p1_max_disp >= 0.30 and 'S2_MOVED' not in self.milestones_detected:
                    self.milestones_detected.add('S2_MOVED')
                    self.logger.milestone("S2_MOVED", f"pinky1 실측 이동거리 {p1_max_disp:.2f}m 돌파 (기준 0.3m 충족)")
                    self.results['S2_closed_loop'] = 'PASS'

                # S3: 교차로 감속/정지/통과
                p1_reason = p1.get('reason', '')
                if ('교차로 정지' in p1_reason or p1.get('drive') == 12) and 'S3_JUNCTION_STOP' not in self.milestones_detected:
                    self.milestones_detected.add('S3_JUNCTION_STOP')
                    self.logger.milestone("S3_JUNCTION_STOP", f"pinky1 교차로 감속/정지 감지: {p1_reason}")

                if holders.get('J') == 'pinky1' and 'S3_JUNCTION_OCCUPIED' not in self.milestones_detected:
                    self.milestones_detected.add('S3_JUNCTION_OCCUPIED')
                    self.logger.milestone("S3_JUNCTION_OCCUPIED", "pinky1 교차로 J 점유 노드 획득 및 통과 시작")
                    self.results['S3_junction'] = 'PASS'

                # S4: 교차로 양보
                p2_blocked = p2.get('blocked_by')
                if p2_blocked == 'pinky1' and 'S4_YIELD' not in self.milestones_detected:
                    self.milestones_detected.add('S4_YIELD')
                    self.logger.milestone("S4_YIELD", f"pinky2 양보 대기 확인 (BL_J 앞에서 {p2_blocked} 통과 대기)")
                    self.results['S4_yield'] = 'PASS'

                # S5: 교차로 점유 인계 (J 노드)
                if holders.get('J') == 'pinky2' and 'S3_JUNCTION_OCCUPIED' in self.milestones_detected and 'S5_HANDOVER' not in self.milestones_detected:
                    self.milestones_detected.add('S5_HANDOVER')
                    self.logger.milestone("S5_HANDOVER", "교차로 J 점유가 pinky1 → pinky2 로 안전하게 인계 완료")
                    self.results['S5_order'] = 'PASS'

                # ── X1: 주행 중 비상정지 및 재개 시험 ──
                if not stop_tested and mission == 'RUNNING':
                    p1_idx = p1.get('route_idx', 0)
                    p2_idx = p2.get('route_idx', 0)
                    if p1_idx >= 2 and p2_idx >= 2 and (abs(p1.get('v', 0)) > 0.02 or abs(p2.get('v', 0)) > 0.02):
                        self.logger.milestone("X1_STOP_TRIGGER", "주행 중 비상정지(STOP) 시험 개시")
                        t_stop_req = time.time()
                        scode, sbody = self.gw.post('/api/fleet/stop')
                        
                        # 속도 0 감속 대기
                        stopped_ok = False
                        for _ in range(15):
                            time.sleep(0.1)
                            _, s_st = self.gw.get('/api/fleet/status')
                            sr = s_st.get('robots', {})
                            if all(abs(sr.get(n, {}).get('v', 0)) < 0.01 for n in ('pinky1', 'pinky2')):
                                deadman_ms = (time.time() - t_stop_req) * 1000.0
                                self.logger.milestone("X1_STOP_CONFIRMED", f"두 로봇 완전 정지 확인 (Deadman 지연: {deadman_ms:.1f}ms)")
                                stopped_ok = True
                                break

                        time.sleep(2.0)
                        self.logger.milestone("X1_RESUME_TRIGGER", "미션 재개(RESUME) 명령 전송")
                        rcode, rbody = self.gw.post('/api/fleet/resume')
                        self.logger.info(f"RESUME 응답: HTTP {rcode} -> {rbody.get('mission_state')}")

                        # 재가속 및 RUNNING 복귀 확인
                        resumed_ok = False
                        for _ in range(25):
                            time.sleep(0.2)
                            _, s_st = self.gw.get('/api/fleet/status')
                            sr = s_st.get('robots', {})
                            if s_st.get('mission') == 'RUNNING' and any(abs(sr.get(n, {}).get('v', 0)) > 0.02 for n in ('pinky1', 'pinky2')):
                                self.logger.milestone("X1_RESUME_CONFIRMED", "정지 후 재출발 및 RUNNING 복귀 실측 확인 (X1 PASS)")
                                resumed_ok = True
                                break

                        if stopped_ok and resumed_ok:
                            self.results['X1_stop_resume'] = 'PASS'
                        else:
                            self.results['X1_stop_resume'] = 'FAIL'
                        stop_tested = True

                # ── S6: 도착 및 구역 이벤트 처리 ──
                for n, r_info in (('pinky1', p1), ('pinky2', p2)):
                    drive_st = r_info.get('drive')
                    goal = r_info.get('goal')
                    # 도착 상태(7: ARRIVED) 및 미션 RUNNING
                    if (drive_st == 7 or r_info.get('arrival') in ('ARRIVED', 'ARRIVAL_CONFIRMED')) and n not in zone_sent:
                        self.logger.milestone("S6_ARRIVED", f"로봇 {n} 목적지({goal}) 도착 보고")
                        zone_sent[n] = True
                        
                        # 태블릿 대행 구역 이벤트 주입 (필요시)
                        if self.args.inject_zone_events and goal:
                            ev_body = {
                                'robot_name': n,
                                'camera_id': f"CAM_{goal}",
                                'zone_id': str(goal),
                                'event_type': 'PRESENT',
                                'confidence': 0.95,
                                'timestamp': time.time(),
                                'sequence': len(zone_sent)
                            }
                            zcode, zresp = self.gw.post('/api/vision/zone_event', ev_body, key=self.vision_key)
                            self.logger.info(f"구역 이벤트 주입 ({n} -> {goal}): HTTP {zcode} ({zresp.get('message', 'SENT')})")

                if len(zone_sent) == 2 and 'S6_ALL_ARRIVED' not in self.milestones_detected:
                    self.milestones_detected.add('S6_ALL_ARRIVED')
                    self.results['S6_arrival'] = 'PASS'

                # ── S7: 미션 완료 (DONE) ──
                if mission == 'DONE':
                    self.logger.milestone("S7_DONE", f"전체 미션 완료 달성! (총 소요 시간: {elapsed:.1f}초)")
                    self.results['S7_done'] = 'PASS'
                    break

                # ── 실시간 콘솔 텔레메트리 롤링 출력 (2Hz) ──
                if now - last_print >= 0.5:
                    last_print = now
                    mins, secs = divmod(int(elapsed), 60)
                    t_clock = f"{mins:02d}:{secs:02d}.{int((elapsed%1)*10)}"
                    
                    p1_line = f"P1:({p1.get('x',0):.2f},{p1.get('y',0):.2f}) v={p1.get('v',0):.2f} #{p1.get('route_idx',0)} [{p1.get('reason','')[:10]}]"
                    p2_line = f"P2:({p2.get('x',0):.2f},{p2.get('y',0):.2f}) v={p2.get('v',0):.2f} #{p2.get('route_idx',0)} [대기:{p2.get('blocked_by','')}]"
                    holder_str = f"J={holders.get('J', 'Free')}"
                    
                    status_line = f"\r{c(f'[{t_clock}]', Colors.YELLOW)} {c(mission, Colors.BOLD)} | {p1_line} | {p2_line} | {holder_str}"
                    sys.stdout.write(status_line.ljust(110))
                    sys.stdout.flush()

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.logger.warn("\n사용자에 의해 모니터링이 중단되었습니다. 긴급 정지 전송...")
            self.gw.post('/api/fleet/stop')

        print() # 개행
        return True

    # =========================================================================
    # Step 5: 영수증 생성 및 서머리 리포트 (Receipt & Summary)
    # =========================================================================
    def step5_generate_receipt(self):
        self.logger.step(5, "테스트 결과 판정 및 최종 영수증 저장")

        # 기본값 채우기
        if 'S2_closed_loop' not in self.results: self.results['S2_closed_loop'] = 'FAIL'
        if 'S3_junction' not in self.results: self.results['S3_junction'] = 'FAIL'
        if 'S4_yield' not in self.results: self.results['S4_yield'] = 'FAIL'
        if 'S5_order' not in self.results: self.results['S5_order'] = 'FAIL'
        if 'S6_arrival' not in self.results: self.results['S6_arrival'] = 'FAIL'
        if 'S7_done' not in self.results: self.results['S7_done'] = 'FAIL'
        if self.args.test_stop and 'X1_stop_resume' not in self.results:
            self.results['X1_stop_resume'] = 'FAIL'
        elif not self.args.test_stop:
            self.results['X1_stop_resume'] = 'SKIP'

        items = [
            ("P0 구성", self.results.get('P0_config', 'FAIL'), "로봇 구성 및 프로파일 일치"),
            ("P1 위치", self.results.get('P1_location', 'FAIL'), "두 로봇 하트비트/위치 신선도"),
            ("S1 시작", self.results.get('S1_start', 'FAIL'), "초기 위치 배정 및 RUNNING 전이"),
            ("S2 폐루프", self.results.get('S2_closed_loop', 'FAIL'), "pinky1 이동거리 0.3m 이상 달성"),
            ("S3 교차로", self.results.get('S3_junction', 'FAIL'), "pinky1 교차로 감속/정지/통과"),
            ("S4 양보", self.results.get('S4_yield', 'FAIL'), "pinky2 교차로 앞 안전 대기"),
            ("S5 순서", self.results.get('S5_order', 'FAIL'), "교차로 점유권 안전 인계"),
            ("S6 도착", self.results.get('S6_arrival', 'FAIL'), "두 로봇 목적지 도착 보고"),
            ("S7 완료", self.results.get('S7_done', 'FAIL'), "전체 미션 DONE 달성"),
            ("X1 정지/재개", self.results.get('X1_stop_resume', 'SKIP'), "비상정지 후 정상 재출발"),
        ]

        total_fails = sum(1 for _, v, _ in items if v == 'FAIL')
        overall_verdict = "PASS" if total_fails == 0 else "FAIL"

        # 콘솔 서머리 테이블 출력
        print("\n" + "=" * 68)
        print(c(f"               현장 실측 영수증 (VERDICT: {overall_verdict})", Colors.BOLD))
        print("=" * 68)
        for name, verdict, desc in items:
            v_color = Colors.GREEN if verdict == 'PASS' else Colors.RED if verdict == 'FAIL' else Colors.YELLOW
            print(f"  {c(verdict.center(7), v_color)} | {name:<12} | {desc}")
        print("=" * 68)
        print(f"  총 10개 평가 항목 중 실패: {c(str(total_fails), Colors.RED if total_fails else Colors.GREEN)}건\n")

        # 영수증 파일 저장
        receipt_data = {
            'verdict': overall_verdict,
            'timestamp': datetime.now().isoformat(),
            'gateway': self.args.gateway,
            'summary': {name: verdict for name, verdict, _ in items},
            'results': items,
            'total_samples': len(self.telemetry_samples)
        }

        receipt_file = self.out_dir / "receipt.json"
        with open(receipt_file, 'w', encoding='utf-8') as f:
            json.dump(receipt_data, f, indent=2, ensure_ascii=False)

        # 텔레메트리 원시 데이터 저장
        raw_file = self.out_dir / "raw.jsonl"
        with open(raw_file, 'w', encoding='utf-8') as f:
            for s in self.telemetry_samples:
                f.write(json.dumps(s, ensure_ascii=False) + '\n')

        self.logger.ok(f"영수증 저장 완료: {receipt_file}")
        self.logger.ok(f"원시 텔레메트리 저장 완료: {raw_file}")
        return overall_verdict == "PASS"

    def run(self):
        try:
            if not self.step0_preflight(): return False
            if not self.step1_robot_discovery(): return False
            if not self.step2_initial_poses(): return False
            if not self.step3_start_mission(): return False
            self.step4_monitor_loop()
            return self.step5_generate_receipt()
        finally:
            self.logger.close()


def main():
    parser = argparse.ArgumentParser(description="중계장비 현장 실측 및 순차 실행 도구")
    parser.add_argument("--gateway", default="http://127.0.0.1:8889", help="중계 관제 게이트웨이 주소 (기본값: http://127.0.0.1:8889)")
    parser.add_argument("--auto", action="store_true", help="단계별 확인 없이 완전 자동 실행")
    parser.add_argument("--test-stop", action="store_true", help="주행 중 임의 STOP & RESUME 재개 시험 수행 (X1 평가)")
    parser.add_argument("--inject-zone-events", action="store_true", help="태블릿 미연결 시 로봇 도착 시 가상 구역 이벤트 자동 발행")
    parser.add_argument("--expected-profile", default="team11_map5_mixed", help="기대 지도 프로파일 이름")
    parser.add_argument("--observe-seconds", type=float, default=180.0, help="최대 관측 시간(초, 기본값: 180)")
    parser.add_argument("--out-dir", default="./field_test_results", help="로그 및 결과물 저장 디렉토리")
    
    args = parser.parse_args()
    runner = FieldTestRunner(args)
    success = runner.run()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stream Enhancer (OpenCV Dynamic Motion Boost & Gazebo Watermark Overlay)
- 동적 영역(이동 로봇) 모션 감지 및 HSV 채도/색상 진하게 강조 (Dynamic Spotlight)
- 정적 배경 톤다운(Muted/Desaturated)을 통한 시각적 간섭 제거
- 가제보(Gazebo) 디지털 트윈 예상 정적 이미지 기반 청사진(Blueprint) 및 반투명 고스트 워터마크 오버레이
"""

import os
import time
import cv2
import numpy as np


class StreamEnhancer:
    """실시간 영상 프레임에 동적 색상 강조 및 가제보 워터마크를 합성하는 프로세서"""

    def __init__(self, gazebo_ref_path=None):
        self.gazebo_ref_path = gazebo_ref_path or os.path.join(
            os.path.dirname(__file__), "static", "gazebo_live_capture.png"
        )
        # 배경 차분 및 모션 검출용 버퍼
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._bg_model = None
        self._last_frame_gray = None
        self._learning_rate = 0.05

        self._gz_ref_img = None
        self._gz_edges = None
        self._load_gazebo_reference()

    def _load_gazebo_reference(self):
        """가제보 기준 정적 이미지 로드 및 외곽선(Canny Edges) 사전 연산"""
        if os.path.exists(self.gazebo_ref_path):
            try:
                img = cv2.imread(self.gazebo_ref_path)
                if img is not None:
                    self._gz_ref_img = img
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    # 벽체 및 주요 장애물 외곽선 추출
                    edges = cv2.Canny(gray, 50, 150)
                    self._gz_edges = cv2.dilate(edges, self._kernel, iterations=1)
            except Exception as e:
                print(f"[StreamEnhancer] Failed to load gazebo reference: {e}")

    def update_gazebo_reference(self, cv_img):
        """실시간 가제보 카메라 영상으로 기준 워터마크 동적 갱신"""
        if cv_img is None:
            return
        self._gz_ref_img = cv_img.copy()
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        self._gz_edges = cv2.dilate(edges, self._kernel, iterations=1)

    def enhance(self, frame, enable_boost=True, enable_watermark=True,
                watermark_mode="blueprint", boost_factor=1.8, bg_desaturate=0.35):
        """
        프레임 강화 처리 메인 함수
        :param frame: 원본 BGR 프레임 (numpy array)
        :param enable_boost: 동적 영역 채도/명도 강조 여부
        :param enable_watermark: 가제보 워터마크 오버레이 여부
        :param watermark_mode: 'blueprint' (형광 외곽선), 'ghost' (반투명), 'hybrid' (둘 다)
        :param boost_factor: 동적 영역 채도 배수 (기본 1.8x)
        :param bg_desaturate: 정적 배경 채도 비율 (기본 0.35x)
        :return: 가공된 BGR 프레임
        """
        if frame is None:
            return None

        out_frame = frame.copy()
        h, w = out_frame.shape[:2]

        # 1. 동적 움직임 영역 색상 진하게 강조 (Dynamic Color Boost)
        if enable_boost:
            out_frame = self._apply_motion_boost(
                out_frame, boost_factor=boost_factor, bg_desaturate=bg_desaturate
            )

        # 2. 가제보 디지털 트윈 워터마크 오버레이 (Gazebo Watermark)
        if enable_watermark and (self._gz_ref_img is not None or self._gz_edges is not None):
            out_frame = self._apply_gazebo_watermark(
                out_frame, h, w, mode=watermark_mode
            )

        return out_frame

    def _apply_motion_boost(self, frame, boost_factor=1.8, bg_desaturate=0.35):
        """움직이는 로봇 영역은 채도를 높이고 정적 배경은 톤다운"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 누적 가중 배경 모델 초기화
        if self._bg_model is None or self._bg_model.shape != gray.shape:
            self._bg_model = np.float32(blurred)
            return frame

        # 배경 모델 점진 갱신 및 차분 연산
        cv2.accumulateWeighted(blurred, self._bg_model, self._learning_rate)
        bg_abs = cv2.convertScaleAbs(self._bg_model)
        diff = cv2.absdiff(blurred, bg_abs)

        # 움직임 마스크 생성
        _, motion_mask = cv2.threshold(diff, 18, 255, cv2.THRESH_BINARY)
        motion_mask = cv2.morphologyEx(motion_mask, cv2.MORPH_OPEN, self._kernel)
        motion_mask = cv2.dilate(motion_mask, self._kernel, iterations=2)

        # HSV 색공간에서 채도(S) 분리 조작
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)

        # A) 정적 배경: 채도 대폭 낮춤 (무채색/흑백화) + 명도 살짝 톤다운
        static_mask = (motion_mask == 0)
        s_ch[static_mask] = (s_ch[static_mask] * bg_desaturate).astype(np.uint8)
        v_ch[static_mask] = (v_ch[static_mask] * 0.85).astype(np.uint8)

        # B) 동적 영역: 채도 1.8배 진하게 부스팅 + 명도 살짝 상향
        dynamic_mask = (motion_mask > 0)
        s_ch[dynamic_mask] = np.clip(s_ch[dynamic_mask].astype(np.float32) * boost_factor, 0, 255).astype(np.uint8)
        v_ch[dynamic_mask] = np.clip(v_ch[dynamic_mask].astype(np.float32) * 1.15, 0, 255).astype(np.uint8)

        boosted = cv2.cvtColor(cv2.merge([h_ch, s_ch, v_ch]), cv2.COLOR_HSV2BGR)

        # C) 움직이는 로봇 주변에 부드러운 형광 외곽선 가이드라인 살짝 추가
        contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 200:  # 미세 노이즈 무시
                cv2.drawContours(boosted, [cnt], -1, (0, 255, 180), 1, cv2.LINE_AA)

        return boosted

    def _apply_gazebo_watermark(self, frame, h, w, mode="blueprint"):
        """가제보 가상환경의 벽체/선반을 워터마크로 오버레이"""
        # 해상도 일치 확인
        gz_edges = self._gz_edges
        if gz_edges is not None and gz_edges.shape[:2] != (h, w):
            gz_edges = cv2.resize(gz_edges, (w, h), interpolation=cv2.INTER_NEAREST)

        # Mode A: 청사진 와이어프레임 엣지 워터마크 (시야 방해 0%)
        if mode in ("blueprint", "hybrid") and gz_edges is not None:
            # 형광 사이언(Cyan: B=255, G=220, R=0) 엣지 합성
            edge_mask = (gz_edges > 0)
            cyan_color = np.array([255, 220, 0], dtype=np.uint8)
            # 반투명 블렌딩
            frame[edge_mask] = cv2.addWeighted(
                frame[edge_mask], 0.35,
                np.tile(cyan_color, (np.count_nonzero(edge_mask), 1)), 0.65, 0
            )

        # Mode B: 반투명 고스트 워터마크 (유령처럼 은은한 배경 오버레이)
        if mode in ("ghost", "hybrid") and self._gz_ref_img is not None:
            gz_img = self._gz_ref_img
            if gz_img.shape[:2] != (h, w):
                gz_img = cv2.resize(gz_img, (w, h), interpolation=cv2.INTER_LINEAR)
            frame = cv2.addWeighted(frame, 0.75, gz_img, 0.25, 0)

        # 워터마크 인디케이터 배지 표시 (우측 상단)
        hud_text = "📐 GAZEBO BLUEPRINT WATERMARK"
        cv2.putText(frame, hud_text, (w - 280, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 220, 0), 1, cv2.LINE_AA)

        return frame

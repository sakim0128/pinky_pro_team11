# -*- coding: utf-8 -*-
"""GET /api/relay/health 의 모양 — gateway_web/relay_health.py (순수, ROS 없이)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'gateway_web'))

import relay_health as RH  # noqa: E402

NOW = 1000.0
ROBOTS = ('pinky1', 'pinky2')


def _link(n=1):
    return {r: {t: n for t in RH.contract_topics(r)} for r in ROBOTS}


def _coord(heard=0.3):
    return {'mission_state': 'ASSIGNED', 'estop_latched': False, 'warning': None, 'profile': {'active': 'map4'},
            'robots': {r: {'last_heard_sec': heard, 'held': False, 'held_reason': None,
                           'map_check': {'state': 'MATCH'}, 'lane_status': {'drive_state': 0, 'state_reason': ''}}
                       for r in ROBOTS}}


def _diag(age=0.5, **agent):
    d = {'nav2': {'active': 9, 'total': 9, 'not_active': []}, 'estop': False, 'gate': {'source': 'MISSION'},
         'tf': {'fresh': True}, 'pose_source': 'AMCL', 'agent': dict({'map_load': 'OK', 'link_lost': False}, **agent)}
    return {r: (d, NOW - age) for r in ROBOTS}


def test_계약_토픽은_live_웹이_보는_다섯이다():
    assert RH.contract_topics('pinky1') == ['/pinky1/state', '/pinky1/lane_status', '/pinky1/diag',
                                             '/pinky1/amcl_pose', '/pinky1/camera/image/compressed']


def test_다_갖추면_ok():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=_coord(), diag=_diag(), control={'view_only': False})
    assert h['ok'] is True and all(r['ok'] for r in h['robots'])
    assert h['coordinator'] == {'present': True, 'mission_state': 'ASSIGNED', 'estop_latched': False,
                                'warning': None, 'profile': 'map4'}
    r = h['robots'][0]
    assert r['diag']['nav2_active'] == 9 and r['diag']['gate_source'] == 'MISSION' and r['diag']['map_load'] == 'OK'


def test_코디네이터가_없으면_ok_가_아니고_이유를_싣는다():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=None, coord_error='ImportError: x', diag=_diag())
    assert h['ok'] is False and h['coordinator'] == {'present': False, 'error': 'ImportError: x'}
    assert [r['name'] for r in h['robots']] == list(ROBOTS)          # 로봇 줄은 그래도 선다


def test_발행자_수를_못_셌으면_None_이지_0_이_아니다():
    h = RH.build(ROBOTS, now=NOW, link=None, coord_status=_coord(), diag=_diag())
    r = h['robots'][0]
    assert r['uplink'] is None and set(r['publishers'].values()) == {None} and r['ok'] is False


def test_발행자가_0_이면_브리지가_안_올린다():
    h = RH.build(ROBOTS, now=NOW, link=_link(0), coord_status=_coord(), diag=_diag())
    assert h['robots'][0]['uplink'] is False and h['ok'] is False


def test_진단은_받은_시각으로_신선도를_잰다():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=_coord(), diag=_diag(age=RH.DIAG_FRESH_S + 0.5))
    d = h['robots'][0]['diag']
    assert d['fresh'] is False and d['age_s'] == round(RH.DIAG_FRESH_S + 0.5, 1) and h['ok'] is False


def test_진단이_없으면_None():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=_coord(), diag={})
    assert h['robots'][0]['diag'] is None and h['ok'] is False


def test_오래_못_들은_로봇은_ok_가_아니다():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=_coord(heard=RH.HEARD_FRESH_S + 1), diag=_diag())
    assert h['ok'] is False


def test_링크유실_래치를_드러낸다():
    h = RH.build(ROBOTS, now=NOW, link=_link(), coord_status=_coord(), diag=_diag(link_lost=True))
    assert h['robots'][0]['diag']['link_lost'] is True


def test_DDS_프로파일은_파일_이름만():
    h = RH.build(ROBOTS, now=NOW, dds_uri='file:///x/y/relay_station/configs/cyclonedds-offsite.xml')
    assert h['dds_profile'] == 'cyclonedds-offsite.xml'

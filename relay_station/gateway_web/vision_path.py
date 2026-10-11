# -*- coding: utf-8 -*-
"""`vision_pose` 가 **어디까지 갔는지**를 말하는 순수 로직 (R-A).

게이트웨이 본체에서 갈라 나왔다. 본체는 module-level 에서 rclpy 를 import 하므로
ROS 없는 곳에서 import 되지 않는다 — **본체 안에 두면 유닛시험이 닿지 못한다.**
(`mjpeg_serving.py` 를 가른 것과 같은 이유다)

## 왜 필요한가 — 수 하나가 두 가지를 뜻했다

`POST /api/vision/pose` 응답의 `hasReceiver` 는 `pub.get_subscription_count()` 에서 온다.
그런데 게이트웨이는 **도메인 8 참여자**이고, 지금 도메인 8 에서 그 토픽을 구독하는 것은
**브리지**다. 2026-09-19 실측:

    D8 /pinky1/vision_pose   Publisher 1 (게이트웨이) · Subscription 1 (pinky_bridge_pinky1_8)
    D10 /vision_pose         Publisher 1 (브리지)     · Subscription 0      ← 여기서 끊긴다

즉 `hasReceiver: true` 는 참이지만 **"로봇이 받는다" 가 아니다.** 연산 노드 세션이 그 값을
종단 근거로 쓸 뻔했고, 관제 세션과 함께 "이 필드를 어느 수락의 근거로도 쓰지 않는다" 로 합의했다.

## 이 모듈이 하는 두 가지

1. **구독자를 누구인지까지 가른다** — 수가 아니라 **이름**을 본다. 브리지인지 아닌지.
2. **하류 계약을 말한다** — 도메인 N 에서 무슨 이름·타입·QoS 로 구독하면 되는지.
   ⭐ 두 번째가 **쓰는 쪽을 위한 것**이다. 2026-09-19 기준 `robots/` 전체에 `vision_pose`
   소비자가 **0 건**이라, 그 코드를 쓸 사람이 브리지 설정을 뒤지지 않아도 되게 응답이 직접 말한다.

⚠️ **못 잰 것은 `None` 이다. 0 이 아니다.** 게이트웨이는 도메인 10 을 볼 수 없으므로
   `subscribers` 를 0 으로 적으면 "소비자가 없다" 는 **다른 주장**이 된다.
"""

# 하류 계약. 출처는 `relay_station/domain_bridge/configs/pinkyN_control.yaml` 의
# `pinkyN/vision_pose` 블록(`remap: vision_pose`, reliable/volatile/keep_last/1).
# 🔴 **리터럴로 적는다** — 포맷 문자열로 만들면 `grep '/vision_pose'` 에 안 걸리고,
#    설정과 어긋났을 때 시험이 대사할 수 없다. 대사는 `test_vision_path` 가 한다.
DOWNSTREAM = {
    'pinky1': {'domain': 10, 'topic': '/vision_pose'},
    'pinky2': {'domain': 11, 'topic': '/vision_pose'},
}
DOWNSTREAM_TYPE = 'geometry_msgs/msg/PoseStamped'
DOWNSTREAM_FRAME = 'map'
DOWNSTREAM_QOS = {
    'reliability': 'reliable', 'durability': 'volatile',
    'history': 'keep_last', 'depth': 1,
}

# 도메인 8 구독자 중 **중간 다리**를 가리는 이름. 브리지 설정의 `name:` 에서 온다
# (`pinky_bridge_pinky1` → 도메인 접미사가 붙어 `pinky_bridge_pinky1_8`).
# ⚠️ 이름으로 가르는 것은 **편의**다. 진실은 `nodes` 목록이고 소비자는 그것을 봐야 한다.
#    이름 규칙이 바뀌면 분류는 틀리지만 `nodes` 는 안 틀린다 — 그래서 둘 다 낸다.
BRIDGE_NODE_PREFIX = 'pinky_bridge_'

NOT_OBSERVABLE = 'NOT_OBSERVABLE_FROM_DOMAIN_8'

# 하류를 재려면 **RMW 와 DDS 설정을 둘 다** 맞춰야 한다. 그 둘을 고르는 곳이
# `domain_bridge/bridge_env.sh` 다(현장/원격 프로파일을 NIC 유무로 고른다).
# 그래서 절차는 그 스크립트를 부르는 형태로 낸다 — 값을 손으로 베끼면 프로파일이 어긋난다.
DOWNSTREAM_RMW = 'rmw_cyclonedds_cpp'
MEASURE_ENV = 'relay_station/domain_bridge/bridge_env.sh'

# 🔴 재는 절차에 걸린 함정 셋. 2026-09-20 에 **이 절차를 실제로 돌리다가** 나왔고,
#    셋 다 "사슬이 끊겼다" 는 거짓 빨강이나 "잘 간다" 는 거짓 초록을 만든다.
#    응답에 같이 실어 보낸다 — 쓰는 쪽이 같은 자리에서 또 넘어지지 않게.
MEASURE_TRAP = (
    '(1) RMW 를 안 맞추면 같은 명령이 "Unknown topic" 을 준다 - 중계는 cyclonedds, '
    'ROS 기본값은 fastrtps. (2) DDS 설정(CYCLONEDDS_URI)까지 맞춰야 한다: 2026-09-20 '
    '중계 실측으로 URI 없이는 6회 중 0회 보였고 bridge_env.sh 를 부르면 6회 중 6회 보였다. '
    '(3) --no-daemon 을 빼면 이미 떠 있는 ros2 daemon 이 호출자의 RMW 와 무관하게 캐시로 '
    '답한다 - 그래서 RMW 를 틀리게 주고도 초록이 났다. '
    '그리고 한 번 보이는 것으로 판정하지 마라: (2) 는 처음 한 번 요행으로 보였다. '
    '경로가 상대경로이니 레포 루트에서 실행한다.'
)


def downstream_contract(robot_id):
    """쓰는 쪽이 그대로 베껴 구독하면 되는 계약. 모르는 것은 `None` 으로 둔다."""
    d = DOWNSTREAM.get(robot_id)
    if d is None:
        return None
    return {
        'domain': d['domain'],
        'topic': d['topic'],
        'type': DOWNSTREAM_TYPE,
        'frameId': DOWNSTREAM_FRAME,
        'qos': dict(DOWNSTREAM_QOS),
        # 🔴 게이트웨이는 도메인 8 참여자다. 아래 둘은 **여기서 잴 수 없다.**
        'subscribers': None,
        'flowHz': None,
        'why': NOT_OBSERVABLE,
        # ⚠️ 환경을 **스크립트로** 세우고 --no-daemon 을 붙인 형태로만 낸다. 하나라도
        #    빠지면 이 명령은 환경에 따라 다른 답을 주고, 쓰는 쪽이 그 답을 사실로 믿는다.
        #    이건 **중계에서** 돌릴 때의 문장이다(`measureFrom`). 로봇 위에서는 그 도메인이
        #    제 집이라 환경이 이미 맞고, 스크립트를 부를 필요가 없다.
        'howToMeasure': ('source %s && ROS_DOMAIN_ID=%d '
                         'ros2 topic info -v --no-daemon %s'
                         % (MEASURE_ENV, d['domain'], d['topic'])),
        'measureFrom': 'RELAY',
        'measureTrap': MEASURE_TRAP,
    }


def classify_receivers(node_names, topic=None, domain=8):
    """도메인 8 구독자 이름 목록을 **브리지 / 그 외**로 가른다.

    `node_names` 가 `None` 이면 **못 잰 것**이다 — 수를 0 으로 만들지 않는다.
    """
    if node_names is None:
        return {'domain': domain, 'topic': topic, 'measured': False,
                'total': None, 'bridge': None, 'consumer': None, 'nodes': None}
    names = sorted(node_names)
    # ⚠️ **노드 이름(마지막 칸)만** 본다. 전체 경로를 보면 네임스페이스가 붙은
    #    `/ns/pinky_bridge_pinky1_8` 을 놓치고, 놓치면 브리지가 **소비자로 집계**돼
    #    "로봇이 받았다" 는 거짓 초록이 난다 — 틀리는 방향이 나쁜 쪽이다.
    #    (첫 판이 `lstrip('/')` 로 판정했고 이 파일의 시험이 배포 전에 잡았다)
    bridge = [n for n in names
              if n.rstrip('/').rsplit('/', 1)[-1].startswith(BRIDGE_NODE_PREFIX)]
    return {
        'domain': domain, 'topic': topic, 'measured': True,
        'total': len(names), 'bridge': len(bridge),
        # ⭐ 종단 판정은 이 값으로 한다 — 브리지가 아닌 구독자가 있는가.
        'consumer': len(names) - len(bridge), 'nodes': names,
    }

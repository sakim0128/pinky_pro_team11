"""상부(천장) 웹캠 노드의 ROS 없는 부분 — 파라미터 검증 · 프레임 → JPEG 인코딩.

overhead_camera_node.py 가 이 함수들을 쓴다. rclpy 없이 pytest 로 돌릴 수 있게 따로 뒀다
(overhead_math.py · overhead_calib.py 와 같은 이유).
"""

DEFAULT_TOPIC = '/overhead/camera/image/compressed'
DEFAULT_FRAME_ID = 'overhead_camera'


def parse_device(value):
    """cv2.VideoCapture 에 줄 장치 지정자.

    int → 그대로(/dev/video<N>). '0' 같은 숫자 문자열 → int. 그 밖의 문자열(경로·GStreamer 파이프라인) → 그대로.
    bool · 음수 · 빈 문자열은 거부한다.
    """
    if isinstance(value, bool):
        raise ValueError(f'device: bool 은 안 된다 ({value!r})')
    if isinstance(value, int):
        if value < 0:
            raise ValueError(f'device: 음수 인덱스 ({value})')
        return value
    if isinstance(value, str):
        text = value.strip()
        if not text:
            raise ValueError('device: 빈 문자열')
        if text.isdigit():
            return int(text)
        return text
    raise ValueError(f'device: int 또는 str 이어야 한다 ({type(value).__name__})')


def validate_params(width, height, fps, jpeg_quality):
    """캡처 크기·주기·JPEG 품질을 검증해 (width, height, fps, jpeg_quality) 로 돌려준다."""
    try:
        width = int(width)
        height = int(height)
        fps = float(fps)
        jpeg_quality = int(jpeg_quality)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'width/height/jpeg_quality 는 int, fps 는 float 이어야 한다: {exc}') from exc
    if width <= 0 or height <= 0:
        raise ValueError(f'width·height 는 양수여야 한다 ({width}x{height})')
    if not (fps > 0.0 and fps == fps and fps != float('inf')):
        raise ValueError(f'fps 는 유한한 양수여야 한다 ({fps})')
    if not 1 <= jpeg_quality <= 100:
        raise ValueError(f'jpeg_quality 는 1..100 ({jpeg_quality})')
    return width, height, fps, jpeg_quality


def encode_jpeg(cv2, frame, jpeg_quality):
    """BGR/그레이 프레임 → JPEG bytes. 프레임이 비었거나 인코딩이 실패하면 None."""
    if frame is None or getattr(frame, 'size', 0) == 0:
        return None
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        return None
    return buf.tobytes()

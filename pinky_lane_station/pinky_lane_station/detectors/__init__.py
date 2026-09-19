"""검출기 레지스트리. 무거운 백엔드(ultralytics/torch)는 요청될 때만 import 한다."""

from .base import Detector, DetectorError

KINDS = ('stub', 'classic', 'ultralytics')


def create_detector(spec):
    """spec: {'kind': 'ultralytics', ...} 또는 kind 문자열. 나머지 키는 백엔드 생성자 인자."""
    if isinstance(spec, str):
        spec = {'kind': spec}
    spec = dict(spec or {})
    kind = str(spec.pop('kind', 'stub')).lower()
    if kind == 'stub':
        from .stub import StubDetector
        return StubDetector(**spec)
    if kind == 'classic':
        from .classic import ClassicDetector
        return ClassicDetector(**spec)
    if kind in ('ultralytics', 'yolo'):
        from .ultralytics_backend import UltralyticsDetector
        return UltralyticsDetector(**spec)
    raise DetectorError(f'알 수 없는 검출기 kind: {kind!r} (가능: {", ".join(KINDS)})')


__all__ = ['Detector', 'DetectorError', 'create_detector', 'KINDS']

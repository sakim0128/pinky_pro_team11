"""Read the small Nav2 PGM map and expose a browser-safe PNG rendition."""
import struct
import zlib
from pathlib import Path

import yaml


class MapAssetError(ValueError):
    """The selected map cannot be safely served to the dashboard."""


def _header(raw):
    """Return PGM header tokens and the first pixel offset.

    PGM comments and whitespace are permitted anywhere between header values, so
    searching for the literal bytes ``255`` is not a safe way to find pixels.
    """
    tokens = []
    index = 0
    size = len(raw)
    while len(tokens) < 4:
        while index < size and raw[index:index + 1] in b' \t\r\n':
            index += 1
        if index < size and raw[index:index + 1] == b'#':
            newline = raw.find(b'\n', index)
            index = size if newline == -1 else newline + 1
            continue
        start = index
        while index < size and raw[index:index + 1] not in b' \t\r\n#':
            index += 1
        if start == index:
            raise MapAssetError('Invalid PGM header')
        tokens.append(raw[start:index])
    if index >= size or raw[index:index + 1] not in b' \t\r\n':
        raise MapAssetError('PGM header is missing the pixel separator')
    while index < size and raw[index:index + 1] in b' \t\r\n':
        index += 1
    return tokens, index


def _pgm(path):
    raw = Path(path).read_bytes()
    tokens, header_end = _header(raw)
    if len(tokens) < 4 or tokens[:1] != [b'P5']:
        raise MapAssetError('P5 PGM map is required')
    try:
        width, height, maximum = map(int, tokens[1:4])
    except ValueError as exc:
        raise MapAssetError('Invalid PGM header') from exc
    if width <= 0 or height <= 0 or maximum != 255:
        raise MapAssetError('Unsupported PGM dimensions or colour depth')
    pixels = raw[header_end:]
    if len(pixels) != width * height:
        raise MapAssetError('PGM pixel data length does not match its header')
    return width, height, pixels


def _png_gray(width, height, pixels):
    scanlines = b''.join(b'\x00' + pixels[row * width:(row + 1) * width]
                         for row in range(height))
    def chunk(kind, payload):
        return (struct.pack('>I', len(payload)) + kind + payload +
                struct.pack('>I', zlib.crc32(kind + payload) & 0xffffffff))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))


class MapAsset:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).resolve()
        try:
            meta = yaml.safe_load(self.yaml_path.read_text(encoding='utf-8'))
            image_name = meta['image']
            self.resolution = float(meta['resolution'])
            self.origin = [float(value) for value in meta['origin']]
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise MapAssetError(f'Invalid map YAML: {exc}') from exc
        if len(self.origin) != 3 or self.resolution <= 0:
            raise MapAssetError('Map origin must have three values and resolution must be positive')
        image_path = Path(image_name)
        self.image_path = image_path if image_path.is_absolute() else self.yaml_path.parent / image_path
        self.width, self.height, pixels = _pgm(self.image_path)
        self.png = _png_gray(self.width, self.height, pixels)
        self.metadata = dict(name=self.yaml_path.stem, width=self.width, height=self.height,
                             resolution=self.resolution, origin=self.origin,
                             world_width=self.width * self.resolution,
                             world_height=self.height * self.resolution,
                             image_y_axis='down', world_y_axis='up')

    def compatibility(self, state):
        if not state:
            return 'state_missing'
        if state.get('map_known') is not True:
            return 'robot_map_unknown'
        checks = ((state.get('map_width') == self.width, 'width'),
                  (state.get('map_height') == self.height, 'height'),
                  (abs(float(state.get('map_resolution', float('nan'))) - self.resolution) < 1e-6, 'resolution'),
                  (abs(float(state.get('map_origin_x', float('nan'))) - self.origin[0]) < 1e-6, 'origin_x'),
                  (abs(float(state.get('map_origin_y', float('nan'))) - self.origin[1]) < 1e-6, 'origin_y'))
        mismatch = [name for matched, name in checks if not matched]
        return 'compatible' if not mismatch else 'mismatch:' + ','.join(mismatch)

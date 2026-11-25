#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate multi-size Windows icon (assets/tunepro.ico) using Python stdlib only."""
from __future__ import annotations

import math
from pathlib import Path
import struct
import zlib


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)


def _encode_png_rgba(size: int, pixels: bytearray) -> bytes:
    raw = bytearray()
    stride = size * 4
    for y in range(size):
        raw.append(0)
        raw.extend(pixels[y * stride:(y + 1) * stride])
    ihdr = struct.pack('>IIBBBBB', size, size, 8, 6, 0, 0, 0)
    idat = zlib.compress(bytes(raw), 9)
    return b'\x89PNG\r\n\x1a\n' + _png_chunk(b'IHDR', ihdr) + _png_chunk(b'IDAT', idat) + _png_chunk(b'IEND', b'')


def _blend(dst: tuple[int, int, int, int], src: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    sa = src[3] / 255.0
    da = dst[3] / 255.0
    oa = sa + da * (1.0 - sa)
    if oa <= 1e-6:
        return (0, 0, 0, 0)
    r = int(round((src[0] * sa + dst[0] * da * (1.0 - sa)) / oa))
    g = int(round((src[1] * sa + dst[1] * da * (1.0 - sa)) / oa))
    b = int(round((src[2] * sa + dst[2] * da * (1.0 - sa)) / oa))
    return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)), max(0, min(255, int(round(oa * 255)))))


def render_icon_png(size: int) -> bytes:
    buf = bytearray(size * size * 4)
    radius = size * 0.22
    pad = size * 0.05
    inner_lo = pad + radius
    inner_hi = size - pad - radius

    def set_px(x: int, y: int, col: tuple[int, int, int, int]) -> None:
        if 0 <= x < size and 0 <= y < size:
            idx = (y * size + x) * 4
            cur = (buf[idx], buf[idx + 1], buf[idx + 2], buf[idx + 3])
            out = _blend(cur, col)
            buf[idx:idx + 4] = bytes(out)

    for y in range(size):
        ny = (y + 0.5) / size
        for x in range(size):
            nx = (x + 0.5) / size
            # Rounded square SDF
            cx = max(inner_lo, min(inner_hi, x + 0.5))
            cy = max(inner_lo, min(inner_hi, y + 0.5))
            dist = math.hypot((x + 0.5) - cx, (y + 0.5) - cy) - radius
            if dist > 1.2:
                continue
            alpha = max(0.0, min(1.0, 0.8 - dist))
            # Warm obsidian gradient + subtle clay top-left mood glow
            glow = max(0.0, 1.0 - math.hypot(nx - 0.28, ny - 0.24) * 1.45)
            r = int(31 + ny * 10 + glow * 42)
            g = int(30 + ny * 9 + glow * 20)
            b = int(28 + ny * 8 + glow * 10)
            # Subtle border highlight
            if -1.8 <= dist <= 0.6:
                r = min(255, r + 28)
                g = min(255, g + 26)
                b = min(255, b + 22)
            set_px(x, y, (r, g, b, int(alpha * 255)))

            # Subtle grid hairline
            if abs(ny - 0.5) < (0.8 / size) and 0.16 < nx < 0.84:
                set_px(x, y, (236, 235, 230, 36))

            # Step-target dashed amber reference line at y=0.33
            if 0.16 <= nx <= 0.84 and abs(ny - 0.34) < (1.1 / size):
                if int(nx * 18) % 2 == 0:
                    set_px(x, y, (226, 178, 89, 210))

            # Primary damped step-response curve (Terracotta clay #d97757)
            if 0.16 <= nx <= 0.84:
                t = (nx - 0.16) / 0.68
                y_clay = 0.72 - 0.38 * (1.0 - math.exp(-5.4 * t) * math.cos(7.2 * t))
                d_clay = abs(ny - y_clay) * size
                stroke = max(1.6, size * 0.042)
                if d_clay <= stroke + 1.0:
                    a = max(0.0, min(1.0, (stroke + 0.8 - d_clay) / 1.4))
                    set_px(x, y, (217, 119, 87, int(a * 255)))

                # Secondary parallel speed curve (Emerald #4fc3a1)
                y_em = 0.76 - 0.25 * (1.0 - math.exp(-4.2 * t))
                d_em = abs(ny - y_em) * size
                stroke2 = max(1.3, size * 0.032)
                if d_em <= stroke2 + 1.0:
                    a2 = max(0.0, min(1.0, (stroke2 + 0.8 - d_em) / 1.4))
                    set_px(x, y, (79, 195, 161, int(a2 * 235)))

            # Indicator dot on upper-right
            d_dot = math.hypot((nx - 0.78) * size, (ny - 0.23) * size)
            dot_r = max(1.8, size * 0.055)
            if d_dot <= dot_r + 1.0:
                a_dot = max(0.0, min(1.0, dot_r + 0.6 - d_dot))
                set_px(x, y, (79, 195, 161, int(a_dot * 255)))

    return _encode_png_rgba(size, buf)


def write_ico(out_path: Path) -> Path:
    sizes = [32, 64, 256]
    pngs = [render_icon_png(s) for s in sizes]
    out_path.parent.mkdir(parents=True, exist_ok=True)

    header = struct.pack('<HHH', 0, 1, len(sizes))
    entries = bytearray()
    offset = 6 + 16 * len(sizes)
    for sz, png in zip(sizes, pngs):
        w = 0 if sz >= 256 else sz
        h = 0 if sz >= 256 else sz
        entries.extend(struct.pack('<BBBBHHII', w, h, 0, 0, 1, 32, len(png), offset))
        offset += len(png)

    out_path.write_bytes(header + bytes(entries) + b''.join(pngs))
    return out_path


if __name__ == '__main__':
    root = Path(__file__).resolve().parent.parent
    out = write_ico(root / 'assets' / 'tunepro.ico')
    print(f'Generated icon: {out} ({out.stat().st_size} B)')

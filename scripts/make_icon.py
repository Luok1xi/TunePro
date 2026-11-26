#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate minimalist geometric icons (assets/tunepro.ico, web/favicon.ico, web/favicon.png) using Python stdlib only."""
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


def _dist_seg(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 <= 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def _sample_bezier_polyline() -> list[tuple[float, float]]:
    # Matches web/favicon.svg: M15 44 C24 44, 26 24, 38.5 24 L46 24 (normalized to 0..1 on 64x64)
    p0 = (15 / 64, 44 / 64)
    p1 = (24 / 64, 44 / 64)
    p2 = (26 / 64, 24 / 64)
    p3 = (38.5 / 64, 24 / 64)
    pts: list[tuple[float, float]] = []
    steps = 36
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        x = u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0]
        y = u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1]
        pts.append((x, y))
    pts.append((46 / 64, 24 / 64))
    return pts


CURVE_PTS = _sample_bezier_polyline()


def _blend_over(dst: list[float], r: float, g: float, b: float, a: float) -> None:
    if a <= 0.0:
        return
    a = min(1.0, a)
    inv = 1.0 - a
    oa = a + dst[3] * inv
    if oa <= 1e-6:
        return
    dst[0] = (r * a + dst[0] * dst[3] * inv) / oa
    dst[1] = (g * a + dst[1] * dst[3] * inv) / oa
    dst[2] = (b * a + dst[2] * dst[3] * inv) / oa
    dst[3] = oa


def _sample_pixel(nx: float, ny: float, size: int) -> tuple[float, float, float, float]:
    # Squircle rounded-rect SDF in normalized [0..1] space (matches rect x=2..62 rx=16 on 64x64)
    pad = 2.0 / 64.0
    rad = 16.0 / 64.0
    lo = pad + rad
    hi = 1.0 - pad - rad
    cx = max(lo, min(hi, nx))
    cy = max(lo, min(hi, ny))
    dist_px = (math.hypot(nx - cx, ny - cy) - rad) * size
    if dist_px > 1.2:
        return (0.0, 0.0, 0.0, 0.0)

    px = [0.0, 0.0, 0.0, 0.0]
    bg_a = max(0.0, min(1.0, 0.65 - dist_px))

    # Diagonal obsidian gradient (#222228 -> #141418 -> #0b0b0e)
    diag = max(0.0, min(1.0, 0.55 * nx + 0.65 * ny - 0.10))
    r = 34.0 * (1.0 - diag) + 11.0 * diag
    g = 34.0 * (1.0 - diag) + 11.0 * diag
    b = 40.0 * (1.0 - diag) + 14.0 * diag

    # Subtle warm vermilion aura around the target lock dot (74%, 37.5%)
    aura = max(0.0, 1.0 - math.hypot(nx - 0.74, ny - 0.375) / 0.38)
    aura_sq = aura * aura
    r += 228.0 * 0.24 * aura_sq
    g += 107.0 * 0.24 * aura_sq
    b += 62.0 * 0.24 * aura_sq
    _blend_over(px, min(255.0, r), min(255.0, g), min(255.0, b), bg_a)

    # Crisp 1.5px optical specular rim (stronger at top-left 318deg light)
    rim_w = max(1.0, size * (1.6 / 64.0))
    if -rim_w - 0.8 <= dist_px <= 0.6:
        rim_edge = 1.0 - abs(dist_px + rim_w * 0.45) / (rim_w * 0.75 + 0.5)
        if rim_edge > 0.0:
            top_light = max(0.08, 0.34 * (1.0 - 0.7 * ny) * (1.0 - 0.35 * nx))
            _blend_over(px, 255.0, 255.0, 255.0, rim_edge * top_light * bg_a)

    # Subtle target reference hairline: (15/64, 24/64) -> (42/64, 24/64)
    d_ref = _dist_seg(nx, ny, 15 / 64, 24 / 64, 42 / 64, 24 / 64) * size
    ref_r = max(0.55, size * (1.0 / 64.0))
    if d_ref <= ref_r + 1.0:
        a_ref = max(0.0, min(1.0, ref_r + 0.55 - d_ref))
        _blend_over(px, 244.0, 243.0, 239.0, a_ref * 0.22 * bg_a)

    # Bold minimalist precision S-curve (#f5f4f0)
    min_d = 1e9
    for i in range(len(CURVE_PTS) - 1):
        x1, y1 = CURVE_PTS[i]
        x2, y2 = CURVE_PTS[i + 1]
        d = _dist_seg(nx, ny, x1, y1, x2, y2)
        if d < min_d:
            min_d = d
    d_curve = min_d * size
    curve_r = max(1.0, size * (2.2 / 64.0))
    if d_curve <= curve_r + 1.0:
        a_curve = max(0.0, min(1.0, curve_r + 0.55 - d_curve))
        _blend_over(px, 245.0, 244.0, 240.0, a_curve * bg_a)

    # Signature vermilion target-lock dot (#e46b3e) at (47.5/64, 24/64)
    d_dot = math.hypot(nx - 47.5 / 64, ny - 24 / 64) * size
    dot_r = max(1.55, size * (4.75 / 64.0))
    if d_dot <= dot_r + 1.0:
        a_dot = max(0.0, min(1.0, dot_r + 0.55 - d_dot))
        hi = max(0.0, 1.0 - math.hypot(nx - 46.2 / 64, ny - 22.6 / 64) * 14.0)
        _blend_over(px, min(255.0, 228.0 + hi * 28.0), min(255.0, 107.0 + hi * 38.0), min(255.0, 62.0 + hi * 30.0), a_dot * bg_a)

    return (px[0], px[1], px[2], px[3])


def render_icon_png(size: int) -> bytes:
    buf = bytearray(size * size * 4)
    # 2x2 SSAA on icons <= 64px for crisp anti-aliased edges in titlebars/tabs
    ss = 2 if size <= 64 else 1
    inv_ss2 = 1.0 / (ss * ss)
    for y in range(size):
        for x in range(size):
            if ss == 1:
                r, g, b, a = _sample_pixel((x + 0.5) / size, (y + 0.5) / size, size)
            else:
                ar = ag = ab = aa = 0.0
                for sy in range(ss):
                    ny = (y + (sy + 0.5) / ss) / size
                    for sx in range(ss):
                        nx = (x + (sx + 0.5) / ss) / size
                        sr, sg, sb, sa = _sample_pixel(nx, ny, size)
                        ar += sr * sa
                        ag += sg * sa
                        ab += sb * sa
                        aa += sa
                a = aa * inv_ss2
                if aa > 1e-6:
                    r, g, b = ar / aa, ag / aa, ab / aa
                else:
                    r = g = b = 0.0
            idx = (y * size + x) * 4
            buf[idx] = max(0, min(255, int(round(r))))
            buf[idx + 1] = max(0, min(255, int(round(g))))
            buf[idx + 2] = max(0, min(255, int(round(b))))
            buf[idx + 3] = max(0, min(255, int(round(a * 255.0))))
    return _encode_png_rgba(size, buf)


def write_ico(out_path: Path, sizes: list[int] | None = None) -> tuple[Path, dict[int, bytes]]:
    sz_list = sizes or [16, 24, 32, 48, 64, 256]
    png_map = {s: render_icon_png(s) for s in sz_list}
    out_path.parent.mkdir(parents=True, exist_ok=True)

    header = struct.pack('<HHH', 0, 1, len(sz_list))
    entries = bytearray()
    offset = 6 + 16 * len(sz_list)
    for sz in sz_list:
        png = png_map[sz]
        w = 0 if sz >= 256 else sz
        h = 0 if sz >= 256 else sz
        entries.extend(struct.pack('<BBBBHHII', w, h, 0, 0, 1, 32, len(png), offset))
        offset += len(png)

    out_path.write_bytes(header + bytes(entries) + b''.join(png_map[s] for s in sz_list))
    return out_path, png_map


if __name__ == '__main__':
    root = Path(__file__).resolve().parent.parent
    ico_out, pngs = write_ico(root / 'assets' / 'tunepro.ico')
    web_ico = root / 'web' / 'favicon.ico'
    web_png = root / 'web' / 'favicon.png'
    web_ico.write_bytes(ico_out.read_bytes())
    web_png.write_bytes(pngs[64])
    print(f'Generated icons: {ico_out} ({ico_out.stat().st_size} B), {web_ico}, {web_png} ({web_png.stat().st_size} B)')

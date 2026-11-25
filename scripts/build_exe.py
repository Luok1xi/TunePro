#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build standalone Windows TunePro.exe via PyInstaller."""
from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    subprocess.run([sys.executable, str(ROOT / 'scripts' / 'make_icon.py')], check=True)
    subprocess.run(
        [
            sys.executable, '-m', 'PyInstaller',
            '--noconfirm',
            '--clean',
            str(ROOT / 'TunePro.spec'),
        ],
        cwd=str(ROOT),
        check=True,
    )
    exe = ROOT / 'dist' / ('TunePro.exe' if sys.platform == 'win32' else 'TunePro')
    if not exe.is_file():
        print(f'Build finished but executable not found at {exe}', file=sys.stderr)
        return 1
    sha = hashlib.sha256(exe.read_bytes()).hexdigest()
    size_mb = exe.stat().st_size / (1024 * 1024)
    print(f'\nBuilt: {exe}')
    print(f'Size : {size_mb:.2f} MB ({exe.stat().st_size} bytes)')
    print(f'SHA256: {sha}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

# -*- mode: python ; coding: utf-8 -*-
# SPDX-License-Identifier: GPL-3.0-or-later
"""PyInstaller spec for building standalone single-file TunePro.exe (zero runtime Python needed)."""
from pathlib import Path

ROOT = Path(SPECPATH).resolve()
ICON = ROOT / 'assets' / 'tunepro.ico'

a = Analysis(
    [str(ROOT / 'app.py')],
    pathex=[str(ROOT)],
    binaries=[],
    datas=[
        (str(ROOT / 'web'), 'web'),
        (str(ROOT / 'skills' / 'tc264-pid-tuner'), 'skills/tc264-pid-tuner'),
        (str(ROOT / 'THIRD_PARTY_NOTICES.md'), '.'),
    ],
    hiddenimports=[
        'serial_bridge',
        'demo_sim',
        'ai_tuner',
        'ssl',
        'winreg',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'tkinter',
        'unittest',
        'test',
        'pydoc',
        'doctest',
        'pdb',
        'xmlrpc',
        'lib2to3',
        'distutils',
        'setuptools',
        'pip',
    ],
    noarchive=False,
    optimize=1,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='TunePro',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(ICON) if ICON.is_file() else None,
)

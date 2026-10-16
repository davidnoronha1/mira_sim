#!/usr/bin/env python3
"""Write /tmp/.mira-sim.xauth so Gazebo's GUI in the container can reach the host X server.

The file is mounted in the containers as /tmp/.docker.xauth (rootless,
network_mode: host, X11 passthrough). Pattern from src/bluerov2_gz/docker/run.sh,
hardened for Wayland/XWayland.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys

XAUTH = Path('/tmp/.mira-sim.xauth')


def xhost_local():
    """Let local containers connect without a cookie (fallback only)."""
    if shutil.which('xhost'):
        for target in ('+local:docker', '+local:'):
            if subprocess.run(['xhost', target], capture_output=True).returncode == 0:
                return


def main():
    # A container started before this ever ran makes Docker create the bind
    # source as a root-owned directory; nothing here can replace it.
    if XAUTH.is_dir():
        sys.exit(f'[x11-setup] ERROR: {XAUTH} is a directory (created by Docker for a missing bind mount).\n'
                 f'[x11-setup] Fix once with:  docker compose down && sudo rmdir {XAUTH}')
    display = os.environ.get('DISPLAY', '')
    if display and shutil.which('xauth'):
        # Always regenerate: stale cookies after a VT switch / display-manager
        # restart cause "Invalid MIT-MAGIC-COOKIE-1".
        XAUTH.unlink(missing_ok=True)
        cookies = subprocess.run(['xauth', 'nlist', display], capture_output=True, text=True).stdout
        # FamilyLocal (0100) -> FamilyWild (ffff): accepted from the container's
        # network namespace too (harmless with host networking).
        wild = ''.join('ffff' + line[4:] + '\n' for line in cookies.splitlines() if line)
        if wild:
            subprocess.run(['xauth', '-f', str(XAUTH), 'nmerge', '-'], input=wild, text=True, capture_output=True)
        # nlist can be empty on Wayland/XWayland with no cookie.
        if not XAUTH.exists() or XAUTH.stat().st_size == 0:
            XAUTH.touch()
            xhost_local()
    else:
        # Headless / pure Wayland / no xauth: an empty cookie file still
        # satisfies the bind mount; Gazebo can render offscreen.
        if not display:
            print('[x11-setup] WARNING: DISPLAY is empty (headless/Wayland?). Gazebo GUI will not show.', file=sys.stderr)
        XAUTH.touch()
        xhost_local()
    try:
        XAUTH.chmod(0o644)
    except OSError:
        pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Loud banner stating whether a bringup runs on the NVIDIA GPU, and why not.

Usage: gpu-banner.py <compose service chosen by bringup.py> [tmux session]
The optional tmux session also gets the verdict in its status bar.
"""
import os
import subprocess
import sys


def quiet(*command):
    try:
        return subprocess.run(command, capture_output=True, text=True)
    except FileNotFoundError:
        return None


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    service, session = sys.argv[1], (sys.argv[2] if len(sys.argv) == 3 else None)
    smi = quiet('nvidia-smi', '-L')
    driver = smi is not None and smi.returncode == 0
    gpu = ''
    if driver:
        names = quiet('nvidia-smi', '--query-gpu=name', '--format=csv,noheader')
        gpu = names.stdout.splitlines()[0] if names and names.stdout else ''
    runtimes = quiet('docker', 'info', '--format', '{{json .Runtimes}}')
    runtime = runtimes is not None and '"nvidia"' in runtimes.stdout

    if service == 'mira-sim-gpu':
        colour, title = '1;42;30', f'GPU ENABLED: {gpu or "NVIDIA (forced by MIRA_GPU=1)"}'
        lines = ['Gazebo rendering, YOLO (CUDA) and GPU-capable nodes use the NVIDIA GPU.',
                 f'Service: {service}.  Force CPU with MIRA_GPU=0.']
        if not (driver and runtime):
            colour, title = '1;41;97', 'GPU FORCED BUT NOT USABLE'
            lines = [f'driver ok: {int(driver)}   docker nvidia runtime: {int(runtime)}',
                     'The GPU container will fail to start. Unset MIRA_GPU to fall back to CPU.']
    else:
        colour, title, hint = '1;43;30', 'GPU NOT AVAILABLE - RUNNING ON CPU', None
        if os.environ.get('MIRA_GPU') == '0':
            reason = 'Disabled by MIRA_GPU=0.'
        elif not driver:
            reason = 'No working NVIDIA driver (nvidia-smi failed).'
        elif not runtime:
            reason = f'Found {gpu}, but Docker has no nvidia runtime.'
            hint = 'Install nvidia-container-toolkit, then: sudo nvidia-ctk runtime configure --runtime=docker'
        else:
            reason = f'Service overridden to {service}.'
        lines = [reason, *([hint] if hint else []),
                 'Gazebo uses software/iGPU rendering; YOLO and visual odometry run on CPU.',
                 'Expect low camera rates; HEADLESS=1 helps.']

    bar = '#' * 104
    print(f'\n\033[{colour}m{bar}\033[0m')
    for line in (title, '', *lines):
        print(f'\033[{colour}m##  {line:<98}##\033[0m')
    print(f'\033[{colour}m{bar}\033[0m\n')

    if session and quiet('tmux', 'has-session', '-t', session).returncode == 0:
        style = 'bg=green,fg=black,bold' if title.startswith('GPU ENABLED') else 'bg=red,fg=white,bold'
        quiet('tmux', 'set-option', '-t', session, 'status-right-length', '80')
        quiet('tmux', 'set-option', '-t', session, 'status-right', f'#[{style}] {title} ')


if __name__ == '__main__':
    main()

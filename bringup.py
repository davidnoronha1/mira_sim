#!/usr/bin/env python3
"""Simulator lifecycle and tmux orchestration. Run --help for commands."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
SESSION = 'mira-sim'
MAPS = {
    'sauvc': ('sauvc_sim', 'sauvc_competition.world', '7', '-10'),
    'sauvc_quali': ('sauvc_sim', 'sauvc25.world', '11', '-11.5'),
    'tacc': ('tacc_sim', 'tacc.world', '0', '0'),
    'bluerov': ('bluerov2_gz', 'bluerov2_heavy_underwater.world', '0', '0'),
}


def run(command, env=None, **kwargs):
    return subprocess.run(command, cwd=ROOT, env=env, check=True, **kwargs)


def gpu_available():
    if not shutil.which('nvidia-smi') or not shutil.which('docker'):
        return False
    driver = subprocess.run(['nvidia-smi', '-L'], capture_output=True)
    runtime = subprocess.run(['docker', 'info', '--format', '{{json .Runtimes}}'], capture_output=True, text=True)
    return driver.returncode == 0 and runtime.returncode == 0 and '"nvidia"' in runtime.stdout


def settings(args):
    if args.resolved:
        return json.loads(args.resolved)
    env = os.environ.copy()
    gpu = env.get('MIRA_GPU', '')
    if gpu not in ('', '0', '1'):
        raise ValueError('MIRA_GPU must be 0 or 1')
    use_gpu = gpu == '1' or (gpu != '0' and gpu_available())
    defaults = {
        'MIRA_ROOT': str(ROOT.parent / 'mira'), 'VISION_MAP': 'sauvc',
        'GZ_SERVICE': 'mira-sim-gpu' if use_gpu else 'mira_sim',
        'HEADLESS': '0', 'SITL_WAIT': '300', 'GZ_PARTITION': 'mira_sim', 'ROS_DOMAIN_ID': '0',
    }
    cfg = {key: env.get(key, value) for key, value in defaults.items()}
    cfg['MODE'] = args.mode
    cfg['VISION_MAP'] = args.map or cfg['VISION_MAP']
    if args.headless:
        cfg['HEADLESS'] = '1'
    cfg['NO_SITL'] = str(int(args.no_sitl))
    if cfg['VISION_MAP'] not in MAPS:
        raise ValueError(f'VISION_MAP must be one of {", ".join(MAPS)}')
    package, world, x, y = MAPS[cfg['VISION_MAP']]
    cfg.update(SIM_PACKAGE=package, VISION_WORLD=f'/workspace/{package}/worlds/{world}', START_X=x, START_Y=y)
    cfg['MIRA_ROOT'] = str(Path(cfg['MIRA_ROOT']).resolve())
    cfg['SITL_SERVICE'] = 'ardupilot-sitl' if package == 'bluerov2_gz' else 'ardupilot-sitl-vision'
    if cfg['GZ_SERVICE'] not in ('mira_sim', 'mira-sim-gpu'):
        raise ValueError('GZ_SERVICE must be mira_sim or mira-sim-gpu')
    if cfg['HEADLESS'] not in ('0', '1'):
        raise ValueError('HEADLESS must be 0 or 1')
    if float(cfg['SITL_WAIT']) <= 0:
        raise ValueError('SITL_WAIT must be positive')
    return cfg


def environment(cfg):
    return {**os.environ, **cfg, 'UID': str(os.getuid()), 'GID': str(os.getgid()),
            '_UID': str(os.getuid()), '_GID': str(os.getgid()), 'MIRA_SIM_ROOT': str(ROOT),
            'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}


def compose(cfg):
    return ['docker', 'compose', '-f', str(ROOT / 'docker-compose.yml')]


def components(cfg):
    names = ['gazebo']
    if cfg['MODE'] != 'gazebo':
        if cfg['NO_SITL'] != '1':
            names += ['sitl']
        names += ['bridge']
    return names


def component_command(name, cfg):
    base = compose(cfg)
    if name == 'sitl':
        return [*base, 'up', cfg['SITL_SERVICE']]
    command = [*base, 'exec', '-T']
    for key, value in cfg.items():
        command += ['-e', f'{key}={value}']
    for key, value in {'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}.items():
        command += ['-e', f'{key}={value}']
    command += [cfg['GZ_SERVICE'], 'bash', '-c']
    if name == 'bridge':
        return [*command, 'source /opt/ros/jazzy/setup.bash && exec ros2 launch '
                '/workspace/dnt_simulator/launch/bridge.launch.py']
    flags = ['-v3', '-r', *(['-s', '--headless-rendering'] if cfg['HEADLESS'] == '1' else [])]
    return [*command, 'export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u); mkdir -p -m 700 "$XDG_RUNTIME_DIR"; '
            'source /tmp/gz-render-env.sh 2>/dev/null || true; exec ' + shlex.join(['gz', 'sim', *flags, cfg['VISION_WORLD']])]


def wait_for_gazebo(timeout):
    deadline = time.monotonic() + timeout
    print(f'Waiting up to {timeout:g}s for Gazebo plugin on UDP 9002...', flush=True)
    while time.monotonic() < deadline:
        for table in ('/proc/net/udp', '/proc/net/udp6'):
            rows = Path(table).read_text().splitlines()[1:]
            if any(int(row.split()[1].rsplit(':', 1)[1], 16) == 9002 for row in rows):
                return
        time.sleep(1)
    raise ValueError('Gazebo plugin did not become ready; inspect the gazebo tmux window')


def window(name, cfg, first=False):
    # Freeze resolved settings; tmux's server may have an older environment.
    cmd = shlex.join([sys.executable, str(ROOT / 'bringup.py'), 'component', name, '--resolved', json.dumps(cfg)])
    if first:
        run(['tmux', 'new-session', '-d', '-s', SESSION, '-n', name], env=environment(cfg))
    else:
        run(['tmux', 'new-window', '-t', SESSION, '-n', name])
    target = f'{SESSION}:{name}'
    run(['tmux', 'set-option', '-w', '-t', target, 'remain-on-exit', 'on'])
    run(['tmux', 'respawn-pane', '-k', '-t', target, cmd])


def prepare_display(cfg):
    if cfg['HEADLESS'] == '1':
        Path('/tmp/.mira-sim.xauth').touch(exist_ok=True)
    else:
        run([sys.executable, str(ROOT / 'misc/infra/x11-setup.py')])


def host_task(task, packages):
    if task == 'submodules':
        run(['git', 'submodule', 'update', '--init', '--recursive'])
        return
    if task == 'clean':
        for name in ('build', 'install', 'log'):
            path = ROOT / name
            if path.exists():
                shutil.rmtree(path)
        return
    if task == 'install-deps':
        run(['uv', 'sync'])
        run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y'])
        return
    command = ['colcon', 'build', '--parallel-workers', str(os.cpu_count() or 1),
               '--event-handlers', 'console_cohesion+', '--continue-on-error']
    if packages:
        command += ['--packages-select', *packages]
    command += ['--cmake-args', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DCMAKE_BUILD_TYPE=RelWithDebInfo']
    run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && exec ' + shlex.join(command)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', choices=('up', 'down', 'build', 'check', 'attach', 'shell', 'sitl',
                                         'host-build', 'install-deps', 'submodules', 'clean', 'component'))
    parser.add_argument('component', nargs='?', choices=('gazebo', 'sitl', 'bridge'))
    parser.add_argument('--mode', choices=('sim', 'gazebo'), default='sim', help='sim starts Gazebo, SITL and sensor bridges')
    parser.add_argument('--map', choices=tuple(MAPS))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-sitl', action='store_true', help='sim mode without autopilot')
    parser.add_argument('--package', action='append', default=[], help='host-build package selection')
    parser.add_argument('--resolved', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.task in ('host-build', 'install-deps', 'submodules', 'clean'):
        host_task(args.task, args.package)
        return
    if args.task == 'attach':
        run(['tmux', 'switch-client' if os.environ.get('TMUX') else 'attach-session', '-t', SESSION])
        return
    cfg = settings(args)
    env, base = environment(cfg), compose(cfg)
    if args.task == 'component':
        if not args.component:
            parser.error('component name required')
        if args.component == 'sitl':
            wait_for_gazebo(float(cfg['SITL_WAIT']))
        os.chdir(ROOT)
        os.execvpe('docker', component_command(args.component, cfg), env)
    if args.task == 'check':
        run([*base, 'config', '--quiet'], env=env)
        for name in components(cfg):
            print(f'{name}: {shlex.join(component_command(name, cfg))}')
        return
    if args.task == 'down':
        run([*base, 'stop', 'mira_sim', 'mira-sim-gpu', 'ardupilot-sitl',
             'ardupilot-sitl-vision'], env=env)
        subprocess.run(['tmux', 'kill-session', '-t', SESSION], check=False)
        return
    if args.task == 'build':
        Path('/tmp/.mira-sim.xauth').touch(exist_ok=True)
        services = [cfg['GZ_SERVICE']]
        if cfg['MODE'] != 'gazebo' and cfg['NO_SITL'] != '1':
            services.append(cfg['SITL_SERVICE'])
        run([*base, 'build', *services], env=env)
        return
    if args.task == 'sitl':
        run([*base, 'up', cfg['SITL_SERVICE']], env=env)
        return
    prepare_display(cfg)
    if args.task == 'shell':
        service = cfg['GZ_SERVICE']
        run([*base, 'up', '-d', service], env=env)
        run([*base, 'exec', service, 'bash'], env=env)
        return
    if not shutil.which('tmux'):
        raise ValueError('tmux is required; install it before bring-up')
    for session in (SESSION, 'mira-stack', 'mira-gz', 'mira-sauvc', 'mira-tacc'):
        if subprocess.run(['tmux', 'has-session', '-t', session], capture_output=True).returncode == 0:
            raise ValueError(f'tmux session {session} already exists; stop it before bring-up')
    running = run([*base, 'ps', '--status', 'running', '--services'], env=env, capture_output=True, text=True).stdout.splitlines()
    selected = {cfg['GZ_SERVICE']}
    conflicts = set(running) & ({'ardupilot-sitl', 'ardupilot-sitl-vision', 'mira_sim',
                                 'mira-sim-gpu'} - selected)
    if conflicts:
        raise ValueError(f'Conflicting services already running: {", ".join(sorted(conflicts))}; stop them before bring-up')
    services = [cfg['GZ_SERVICE']]
    run([*base, 'up', '-d', *services], env=env)
    for index, name in enumerate(components(cfg)):
        window(name, cfg, first=index == 0)
    run([sys.executable, str(ROOT / 'misc/infra/gpu-banner.py'), cfg['GZ_SERVICE'], SESSION], env=env)
    print(f'Started {SESSION}. Logs: python3 bringup.py attach; stop: python3 bringup.py down')
    print('Connect Mira independently: make guided-sim '
          f'ARGS="start_x:={cfg["START_X"]} start_y:={cfg["START_Y"]}"')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)

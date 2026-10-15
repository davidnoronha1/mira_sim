#!/usr/bin/env python3
"""Bring up the real Mira packages inside Gazebo's ROS/network environment."""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SESSION = 'mira-stack'
MAPS = {
    'sauvc': ('/workspace/worlds/sauvc_competition.world', '7', '-10'),
    'sauvc_quali': ('/workspace/sauvc_sim/worlds/sauvc25.world', '11', '-11.5'),
    'tacc': ('/workspace/worlds/tacc.world', '0', '0'),
}


def run(args, **kwargs):
    return subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def settings():
    env = os.environ.copy()
    env['MIRA_ROOT'] = str(Path(env.get('MIRA_ROOT', ROOT.parent / 'mira')).resolve())
    env['MIRA_STACK_BUILD'] = str(Path(env.get('MIRA_STACK_BUILD', ROOT / '.mira-stack')).resolve())
    # Compose runs as ${UID}:${GID}; bash does not export UID and nothing sets GID,
    # so without these the container falls back to 1000:1000 and cannot write /mira_build.
    env.setdefault('UID', str(os.getuid()))
    env.setdefault('GID', str(os.getgid()))
    # gz-transport's default partition is hostname:username; a host UID with no
    # passwd entry in the image breaks discovery (GUI never sees the server's
    # /gazebo/starting_world, bridges see no topics). Pin one for every process.
    env.setdefault('GZ_PARTITION', 'mira_stack')
    env.setdefault('GZ_SERVICE', 'mira_sim')
    env.setdefault('VISION_MAP', 'sauvc')
    env.setdefault('VISION_SOURCE', 'vo')
    env.setdefault('HEADLESS', '0')
    env.setdefault('RUN_ACTIONS', '1')
    env.setdefault('AUTO_ARM', 'true')
    env.setdefault('TARGET_CLASS', 'dock')
    env.setdefault('ACTION_WAIT', '120')
    env.setdefault('VISION_CONFIG', '/mira/src/mira2_vision/config/guided_front.yaml')
    env.setdefault('VO_BACKEND', 'auto')
    # gazebo: exact boxes from the simulator; yolo: the trained model under test.
    env.setdefault('SIM_DETECTOR', 'gazebo')
    if env['VISION_MAP'] not in MAPS or env['VISION_SOURCE'] not in ('vo', 'gt'):
        raise ValueError('Invalid VISION_MAP or VISION_SOURCE')
    if env['GZ_SERVICE'] not in ('mira_sim', 'mira-sim-gpu'):
        raise ValueError('GZ_SERVICE must be mira_sim or mira-sim-gpu')
    for key, values in (('VO_BACKEND', ('auto', 'cuvslam', 'rtabmap')), ('SIM_DETECTOR', ('gazebo', 'yolo')), ('HEADLESS', ('0', '1')), ('RUN_ACTIONS', ('0', '1')), ('AUTO_ARM', ('true', 'false'))):
        if env[key] not in values:
            raise ValueError(f'{key} must be one of {values}')
    env['STACK_SERVICE'] = {'mira_sim': 'mira-stack', 'mira-sim-gpu': 'mira-stack-gpu'}[env['GZ_SERVICE']]
    return env


def compose(env):
    return ['docker', 'compose', '-f', 'docker-compose.yml', '-f', 'docker-compose.mira.yml']


def window_command(component, env):
    keys = ('MIRA_ROOT', 'MIRA_STACK_BUILD', 'GZ_SERVICE', 'VISION_MAP', 'VISION_SOURCE',
            'HEADLESS', 'RUN_ACTIONS', 'AUTO_ARM', 'TARGET_CLASS', 'ACTION_WAIT', 'VISION_CONFIG', 'VO_BACKEND',
            'GZ_PARTITION', 'SIM_DETECTOR')
    return shlex.join(['env', *[f'{key}={env[key]}' for key in keys], sys.executable,
                       str(Path(__file__).resolve()), 'component', component])


GAZEBO_JSON_PORT = 9002  # ArduPilotPlugin's lockstep socket (--model JSON:127.0.0.1)


def udp_port_bound(port):
    for table in ('/proc/net/udp', '/proc/net/udp6'):
        try:
            rows = Path(table).read_text().splitlines()[1:]
        except OSError:
            continue
        if any(int(row.split()[1].rsplit(':', 1)[1], 16) == port for row in rows):
            return True
    return False


def wait_for_gazebo(timeout):
    """ArduSub must start after the plugin listens, or the lockstep starts in a bad state."""
    deadline = time.monotonic() + timeout
    print(f'Waiting up to {timeout:.0f}s for Gazebo ArduPilotPlugin on UDP {GAZEBO_JSON_PORT}...', flush=True)
    while not udp_port_bound(GAZEBO_JSON_PORT):
        if time.monotonic() > deadline:
            print('Gazebo plugin never started listening; starting SITL anyway', flush=True)
            return
        time.sleep(1)
    print('Gazebo plugin is listening; starting SITL', flush=True)


def component_command(component, env):
    base = compose(env)
    service = env['STACK_SERVICE']
    world, x, y = MAPS[env['VISION_MAP']]
    if component == 'sitl':
        return [*base, 'up', '--no-recreate', 'ardupilot-sitl-vision']
    command = [*base, 'exec', '-T', '-e', f'GZ_PARTITION={env["GZ_PARTITION"]}']
    if component in ('guided', 'actions'):
        for key, value in {'START_X': x, 'START_Y': y, 'VISION_SOURCE': env['VISION_SOURCE'],
                           'AUTO_ARM': env['AUTO_ARM'], 'TARGET_CLASS': env['TARGET_CLASS'],
                           'ACTION_WAIT': env['ACTION_WAIT'], 'VISION_CONFIG': env['VISION_CONFIG'],
                           'VO_BACKEND': env['VO_BACKEND'], 'SIM_DETECTOR': env['SIM_DETECTOR']}.items():
            command += ['-e', f'{key}={value}']
        return [*command, service, 'bash', '/workspace/mira-stack/mira-stack-runner.sh', component]
    if component == 'gazebo':
        flags = '-v3 -r -s --headless-rendering' if env['HEADLESS'] == '1' else '-v3 -r'
        return [*command, service, 'bash', '-c',
                'export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u); mkdir -p -m 700 "$XDG_RUNTIME_DIR"; '
                f'source /tmp/gz-render-env.sh 2>/dev/null || true; exec gz sim {flags} {shlex.quote(world)}']
    if component == 'bridge':
        return [*command, service, 'bash', '/workspace/vision_nav/bridge.sh']
    raise ValueError(component)


def check_source(env):
    for package in ('mira2_actions', 'mira2_hardware', 'mira2_vision'):
        if not (Path(env['MIRA_ROOT']) / 'src' / package / 'package.xml').is_file():
            raise ValueError(f'MIRA_ROOT does not contain {package}: {env["MIRA_ROOT"]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', choices=('build', 'up', 'actions', 'down', 'check', 'component'))
    parser.add_argument('component', nargs='?', choices=('sitl', 'gazebo', 'bridge', 'guided', 'actions'))
    args = parser.parse_args()
    env = settings()
    base = compose(env)
    if args.task == 'component':
        if not args.component:
            parser.error('component name required')
        os.chdir(ROOT)
        if args.component == 'sitl':
            wait_for_gazebo(float(env.get('SITL_WAIT', '300')))
        os.execvpe('docker', component_command(args.component, env), env)
    if args.task == 'down':
        subprocess.run(['tmux', 'kill-session', '-t', SESSION], cwd=ROOT, check=False)
        run([*base, 'stop', env['STACK_SERVICE'], 'ardupilot-sitl-vision'], env=env)
        return
    check_source(env)
    if args.task == 'check':
        run([*base, 'config', '--quiet'], env=env)
        for item in ('sitl', 'gazebo', 'bridge', 'guided', 'actions'):
            print(shlex.join(component_command(item, env)))
        return
    if args.task == 'build':
        Path('/tmp/.mira-sim.xauth').touch(exist_ok=True)
        Path(env['MIRA_STACK_BUILD']).mkdir(parents=True, exist_ok=True)
        run([*base, 'build', env['STACK_SERVICE']], env=env)
        run([*base, 'run', '--rm', '--no-deps', '--entrypoint', 'bash', env['STACK_SERVICE'],
             '/workspace/mira-stack/mira-stack-runner.sh', 'build'], env=env)
        return
    if not shutil.which('tmux'):
        raise ValueError('tmux is required for integrated bring-up')
    if args.task == 'actions':
        run(['tmux', 'new-window', '-t', SESSION, '-n', 'actions', window_command('actions', env)], env=env)
        return
    for name in (SESSION, 'mira-vision', 'mira-gz', 'mira-sauvc', 'mira-tacc'):
        if subprocess.run(['tmux', 'has-session', '-t', name], capture_output=True).returncode == 0:
            raise ValueError(f'Session {name} already exists; stop it before starting another command owner')
    if not (Path(env['MIRA_STACK_BUILD']) / 'install/setup.bash').is_file():
        raise ValueError('Run make mira-stack-build first')
    # The base compose bind requires this cookie even for headless runs.
    if env['HEADLESS'] == '1':
        Path('/tmp/.mira-sim.xauth').touch(exist_ok=True)
    else:
        run(['bash', 'docker/x11-setup.sh'], env=env)
    running = run([*base, 'ps', '--status', 'running', '--services'], env=env,
                  capture_output=True, text=True).stdout.splitlines()
    if any(service in running for service in ('ardupilot-sitl', 'ardupilot-sitl-vision')):
        raise ValueError('An ArduSub SITL service is already running; stop it before integrated bring-up')
    run([*base, 'up', '--no-recreate', '-d', env['STACK_SERVICE']], env=env)
    run(['tmux', 'new-session', '-d', '-s', SESSION, '-n', 'sitl', window_command('sitl', env)], env=env)
    for item in ('gazebo', 'bridge', 'guided'):
        run(['tmux', 'new-window', '-t', SESSION, '-n', item, window_command(item, env)], env=env)
    if env['RUN_ACTIONS'] == '1':
        run(['tmux', 'new-window', '-t', SESSION, '-n', 'actions', window_command('actions', env)], env=env)
    print(f'Started {SESSION}. Attach: tmux attach -t {SESSION}; stop: make mira-stack-down')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)

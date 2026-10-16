"""Verify that the simulator owns no stack processes or containers."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import bringup


def config(mode='sim', **overrides):
    args = argparse.Namespace(resolved=None, mode=mode, map=None, headless=False, no_sitl=False)
    with patch.dict(os.environ, {'MIRA_GPU': '0', **overrides}, clear=True):
        return bringup.settings(args)


class BringupTests(unittest.TestCase):
    def test_simulator_only_topology(self):
        cfg = config()
        self.assertEqual(bringup.components(cfg), ['gazebo', 'sitl', 'bridge'])
        commands = [bringup.component_command(name, cfg) for name in bringup.components(cfg)]
        self.assertEqual(commands[1][-1], 'ardupilot-sitl-vision')
        self.assertIn('mira_sim', commands[0])
        self.assertIn('mira_sim', commands[2])
        self.assertNotIn('STACK_SERVICE', cfg)
        self.assertNotIn('docker-compose.mira.yml', ' '.join(bringup.compose(cfg)))

    def test_navigation_bridge_always_available_without_stack(self):
        command = bringup.component_command('bridge', config())
        self.assertIn('/workspace/dnt_simulator/launch/bridge.launch.py', command[-1])
        self.assertIn('ROS_DOMAIN_ID=0', command)

    def test_map_selection(self):
        cfg = config(VISION_MAP='sauvc_quali')
        self.assertEqual(cfg['SIM_PACKAGE'], 'sauvc_sim')
        self.assertEqual((cfg['START_X'], cfg['START_Y']), ('11', '-11.5'))
        self.assertEqual(config(VISION_MAP='bluerov')['SITL_SERVICE'], 'ardupilot-sitl')

    def test_frozen_settings_for_tmux(self):
        cfg = config(ROS_DOMAIN_ID='79')
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '0'}):
            self.assertEqual(bringup.settings(argparse.Namespace(resolved=json.dumps(cfg))), cfg)

    def test_component_selection(self):
        self.assertEqual(bringup.components(config(mode='gazebo')), ['gazebo'])
        cfg = config()
        cfg['NO_SITL'] = '1'
        self.assertEqual(bringup.components(cfg), ['gazebo', 'bridge'])

    def test_gazebo_timeout_fails(self):
        with patch('bringup.time.monotonic', side_effect=[0, 2]):
            with self.assertRaisesRegex(ValueError, 'did not become ready'):
                bringup.wait_for_gazebo(1)

    def test_dead_panes_remain_visible(self):
        with patch('bringup.run') as run:
            bringup.window('bridge', config())
        calls = [call.args[0] for call in run.call_args_list]
        self.assertIn(['tmux', 'set-option', '-w', '-t', f'{bringup.SESSION}:bridge', 'remain-on-exit', 'on'], calls)
        self.assertEqual(calls[-1][1], 'respawn-pane')

    def test_up_starts_no_mira_container(self):
        completed = subprocess.CompletedProcess([], 0, stdout='')
        with patch.object(sys, 'argv', ['bringup.py', 'up', '--headless']), \
             patch.dict(os.environ, {'MIRA_GPU': '0'}, clear=True), \
             patch('bringup.shutil.which', return_value='/bin/tmux'), \
             patch('bringup.subprocess.run', return_value=subprocess.CompletedProcess([], 1)), \
             patch('bringup.prepare_display'), patch('bringup.window') as window, \
             patch('bringup.run', return_value=completed) as run:
            bringup.main()
        start = next(call.args[0] for call in run.call_args_list if 'up' in call.args[0])
        self.assertEqual(start[-3:], ['up', '-d', 'mira_sim'])
        self.assertEqual([call.args[0] for call in window.call_args_list], ['gazebo', 'sitl', 'bridge'])

    def test_down_leaves_mira_running(self):
        with patch.object(sys, 'argv', ['bringup.py', 'down']), \
             patch.dict(os.environ, {'MIRA_GPU': '0'}, clear=True), \
             patch('bringup.subprocess.run'), patch('bringup.run') as run:
            bringup.main()
        command = run.call_args.args[0]
        self.assertEqual(command[command.index('stop') + 1:],
                         ['mira_sim', 'mira-sim-gpu', 'ardupilot-sitl', 'ardupilot-sitl-vision'])


if __name__ == '__main__':
    unittest.main()

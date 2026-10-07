#!/usr/bin/env python3
"""Fly ArduSub through a list of local NED waypoints in GUIDED mode.

Waypoints are metres from the EKF origin (north, east, down), with an
optional 4th element for heading in degrees. YAML format:

  acceptance_radius: 0.5   # m, 3D distance to count a waypoint as reached
  hold: 3                  # s to stay inside the radius before moving on
  timeout: 120             # s per waypoint before giving up on it
  waypoints:
    - [0, 0, 1]
    - [2, 0, 1, 90]

Needs a horizontal position estimate (EKF_STATUS_REPORT abs-position flag),
which with GPS disabled comes from vision_bridge.py.
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault('MAVLINK20', '1')

import yaml
from pymavlink import mavutil

M = mavutil.mavlink
# SET_POSITION_TARGET_LOCAL_NED type_mask: ignore vel, accel, yaw rate (+ yaw)
MASK_POS = 0b110111111000
MASK_POS_YAW = 0b100111111000


def wait_autopilot_heartbeat(mav):
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True)
        if hb.autopilot != M.MAV_AUTOPILOT_INVALID:
            mav.target_system, mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return hb


def set_interval(mav, msg_id, hz):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 1e6 / hz, 0, 0, 0, 0, 0)


def wait_position_ok(mav, timeout):
    print('Waiting for EKF horizontal position (vision_bridge running?) ...')
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if msg and msg.flags & M.EKF_POS_HORIZ_ABS:
            print('EKF position OK')
            return
    sys.exit('EKF never reported a horizontal position - check vision_bridge / VISO params')


def set_mode(mav, name, timeout=10):
    # Looked up directly rather than via mav.mode_mapping(), which keys off the
    # last heartbeat's MAV_TYPE and gets confused by vision_bridge's
    # onboard-controller heartbeats sharing sysid 1.
    mode_id = {v: k for k, v in mavutil.mode_mapping_sub.items()}[name]
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.set_mode(mode_id)
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.autopilot != M.MAV_AUTOPILOT_INVALID and hb.custom_mode == mode_id:
            print(f'Mode {name}')
            return
    sys.exit(f'Could not switch to {name} (ArduSub rejects GUIDED without a position estimate)')


def arm(mav, timeout=15):
    mav.arducopter_arm()
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.autopilot != M.MAV_AUTOPILOT_INVALID and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
            print('Armed')
            return
        mav.arducopter_arm()
    sys.exit('Arming failed - check the SITL console for the pre-arm message')


def send_target(mav, n, e, d, yaw_deg):
    mask = MASK_POS if yaw_deg is None else MASK_POS_YAW
    yaw = 0.0 if yaw_deg is None else math.radians(yaw_deg)
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component, M.MAV_FRAME_LOCAL_NED, mask,
        n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)


def fly_to(mav, wp, radius, hold, timeout):
    n, e, d = wp[:3]
    yaw_deg = wp[3] if len(wp) > 3 else None
    start = time.time()
    inside_since = None
    last_send = 0.0
    last_print = 0.0
    while time.time() - start < timeout:
        now = time.time()
        if now - last_send > 0.5:
            send_target(mav, n, e, d, yaw_deg)
            last_send = now
        pos = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if pos is None:
            continue
        dist = math.dist((pos.x, pos.y, pos.z), (n, e, d))
        if now - last_print > 2:
            print(f'  at ({pos.x:+.2f}, {pos.y:+.2f}, {pos.z:+.2f})  dist {dist:.2f} m')
            last_print = now
        if dist <= radius:
            inside_since = inside_since or now
            if now - inside_since >= hold:
                return True
        else:
            inside_since = None
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('waypoints', help='YAML waypoint file')
    ap.add_argument('--mavlink', default='udpin:0.0.0.0:14556')
    ap.add_argument('--disarm', action='store_true', help='disarm after the last waypoint')
    args = ap.parse_args()

    with open(args.waypoints) as f:
        plan = yaml.safe_load(f)
    radius = plan.get('acceptance_radius', 0.5)
    hold = plan.get('hold', 3)
    timeout = plan.get('timeout', 120)

    mav = mavutil.mavlink_connection(args.mavlink, source_system=255)
    print(f'Waiting for ArduSub heartbeat on {args.mavlink} ...')
    wait_autopilot_heartbeat(mav)
    set_interval(mav, M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    set_interval(mav, M.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)

    wait_position_ok(mav, timeout=60)
    set_mode(mav, 'GUIDED')
    arm(mav)

    for i, wp in enumerate(plan['waypoints'], 1):
        print(f'Waypoint {i}/{len(plan["waypoints"])}: {wp}')
        if fly_to(mav, wp, radius, hold, timeout):
            print(f'Reached waypoint {i}')
        else:
            print(f'Timed out on waypoint {i}, moving on')

    if args.disarm:
        mav.arducopter_disarm()
        print('Disarmed')
    else:
        print('Done - holding last waypoint in GUIDED')


if __name__ == '__main__':
    main()

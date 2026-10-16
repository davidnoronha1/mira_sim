"""Fail integrated startup when the running SITL lacks active BendyRuler."""
import os
import time
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

link = mavutil.mavlink_connection('udpin:0.0.0.0:14556', source_system=252)
required = {'OA_TYPE': 1, 'PRX1_TYPE': 2, 'WP_YAW_BEHAVIOR': 1, 'EK3_SRC1_YAW': 6}
found = {}
deadline = time.monotonic() + 60
last_request = 0
try:
    while time.monotonic() < deadline and len(found) < len(required):
        now = time.monotonic()
        if now-last_request > 1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            for name in required:
                if name not in found:
                    link.mav.param_request_read_send(1, 1, name.encode(), -1)
            last_request = now
        message = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=.1)
        if message and message.get_srcSystem() == 1 and message.param_id in required:
            found[message.param_id] = message.param_value
    if found != required:
        raise SystemExit(f'SITL avoidance preflight failed: expected {required}, received {found}. '
                         'Build/recreate the patched SITL service with python3 bringup.py build.')
    print(f'SITL avoidance verified: {found}', flush=True)
finally:
    link.close()

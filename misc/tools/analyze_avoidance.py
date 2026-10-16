"""Extract actual BendyRuler/proximity activity from an ArduPilot DataFlash log."""
import json
import sys
from pathlib import Path
from pymavlink import mavutil


def main():
    log = mavutil.mavlink_connection(sys.argv[1])
    planner, parameters, proximity = [], {}, 0
    while True:
        message = log.recv_match(type=['OABR', 'PARM', 'PRX'])
        if message is None:
            break
        if message.get_type() == 'OABR':
            planner.append(message.to_dict())
        elif message.get_type() == 'PRX':
            proximity += 1
        elif message.Name.startswith(('OA_', 'PRX', 'WP_YAW')):
            parameters[message.Name] = message.Value
    active = [row for row in planner if row['Act']]
    # The Sub patch does not schedule PRX DataFlash logging. MAVProxy's tlog
    # records the actual companion packets received on its sensor link instead.
    transport = mavutil.mavlink_connection(sys.argv[3])
    obstacle_packets = obstacle_returns = 0
    while True:
        message = transport.recv_match(type='OBSTACLE_DISTANCE')
        if message is None:
            break
        if message.get_srcSystem() == 1 and message.get_srcComponent() == 196:
            obstacle_packets += 1
            obstacle_returns += any(message.min_distance <= d <= message.max_distance
                                    for d in message.distances)
    evidence = {'parameters': parameters, 'bendy_ruler_samples': len(planner),
                'active_avoidance_samples': len(active), 'proximity_log_samples': proximity,
                'received_depth_obstacle_packets': obstacle_packets,
                'received_packets_with_obstacle_returns': obstacle_returns,
                'active_desired_yaw_degrees': [row['DYaw'] for row in active],
                'planner_log': planner}
    output = Path(sys.argv[2])
    output.write_text(json.dumps(evidence, indent=2))
    print(json.dumps({k: v for k, v in evidence.items() if k != 'planner_log'}, indent=2))
    if not active or not obstacle_returns or parameters.get('OA_TYPE') != 1 or parameters.get('PRX1_TYPE') != 2:
        raise SystemExit('Missing proof of enabled BendyRuler, proximity or active avoidance')


if __name__ == '__main__':
    main()

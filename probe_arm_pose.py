#!/usr/bin/env python3
import sys, time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

def probe(channel):
    cfg = create_agx_arm_config(robot=ArmModel.NERO, comm='can',
                                channel=channel, firmeware_version=NeroFW.V111)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    arm.set_normal_mode()
    arm.enable()
    start = time.time()
    while time.time() - start < 6:
        a = arm.get_joint_angles()
        if a is not None and a.msg:
            arm.disconnect()
            return list(a.msg)
        time.sleep(0.05)
    arm.disconnect()
    return None

for ch in ('can1', 'can0'):
    print(f'=== probing {ch} ===')
    ang = probe(ch)
    if ang:
        deg = [round(x*180/3.141592653589793, 3) for x in ang]
        print('rad:', [round(x, 4) for x in ang])
        print('deg:', deg)
        print('new_home_joints =', repr([round(x, 4) for x in ang]))
        sys.exit(0)
print('ERROR: no joint feedback on can0/can1')

#! /usr/bin/env python

import time
import yarp
import roboticslab_kinematics_dynamics as kd

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('[error] Please try running yarp server')
    raise SystemExit

options = yarp.Property()
options.put('device', 'CartesianControlClient')
options.put('remote', '/teoSim/rightArm/CartesianControl')
options.put('local', '/cartesianControlExample')

dd = yarp.PolyDriver(options)

if not dd.isValid():
    print('Cannot open the device!')
    raise SystemExit

cc = kd.viewICartesianControl(dd)

print('> getParameters')
params = kd.ConfigMap()
ret = cc.getParameters(params)

# TODO: uncomment when ConfigMap is iterable
# for key, value in params.items():
#     print(f'< {yarp.decode(int(key))}: {value}')

print('> setParameters')
params[kd.ICartesianControl.Config_TRAJ_DURATION] = 5.0
cc.setParameters(params)

print('> getParameter')
ret, value = cc.getParameter(kd.ICartesianControl.Config_TRAJ_DURATION)
print('<', value)

print('> setParameter')
cc.setParameter(kd.ICartesianControl.Config_TRAJ_DURATION, 6.0)

print('> getState')
ret, x, state, ts = cc.getState()
print('<', yarp.decode(state), '[%s]' % ', '.join(map(str, x)), ts)

xd = [
    [0.4025, -0.3469, 0.1692, 0.0, 1.5708, 0.0],
    [0.5, -0.3469, 0.1692, 0.0, 1.5708, 0.0],
    [0.5, -0.4, 0.1692, 0.0, 1.5708, 0.0],
    [0.5, -0.4, 0.1692, 0.0, 1.36, 0.0],
    [0.5, -0.4, 0.1692, 0.6139, 1.4822, 0.6139],
    [0.4025, -0.3469, 0.1692, 0.0, 1.5708, 0.0],
    [0.0, -0.3469, -0.2333, 0.0, 3.1416, 0.0]
]

for i in range(len(xd)):
    print('-- movement ' + str(i + 1) + ':')
    print('> solvePose [%s]' % ', '.join(map(str, xd[i])))
    ret, qd = cc.solvePose(xd[i])

    if ret:
        print('< [%s]' % ', '.join(map(str, qd)))
    else:
        print('< [fail]')
        continue

    print('> movej [%s]' % ', '.join(map(str, xd[i])))

    if cc.moveJoint(xd[i]):
        print('< [ok]')
        print('< [wait...]')

        while True:
            time.sleep(0.1)
            ret, x, state, ts = cc.getState()

            if state == kd.ICartesianControl.State_MOVEJ:
                print('< [moving...]')
            else:
                print('< [done]')
                break
    else:
        print('< [fail]')

print('bye!')

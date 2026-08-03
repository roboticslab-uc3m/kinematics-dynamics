#! /usr/bin/env python

import yarp
import roboticslab_kinematics_dynamics as kd

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('[error] Please try running yarp server')
    raise SystemExit

options = yarp.Property()
options.put('device', 'CartesianControlClient')
options.put('cartesianRemote', '/teoSim/rightArm/CartesianControl')
options.put('cartesianLocal', '/cartesianControlExample')

dd = yarp.PolyDriver(options)

if not dd.isValid():
    print('Cannot open the device!')
    raise SystemExit

cc = kd.viewICartesianControl(dd)

print('> getParameters')
params = kd.IntDoubleMap()
ret = cc.getParameters(params)

for key, value in params.items():
    print(f'< {key}: {value}')

print('> setParameters')
params[kd.VOCAB_CC_CONFIG_TRAJ_DURATION] = 5.0
cc.setParameters(params)

print('> getParameter')
ret, value = cc.getParameter(kd.VOCAB_CC_CONFIG_TRAJ_DURATION)
print('<', value)

print('> setParameter')
cc.setParameter(kd.VOCAB_CC_CONFIG_TRAJ_DURATION, 6.0)

print('> stat')
x = yarp.DVector()
ret, state, ts = cc.stat(x)
print('<', yarp.decode(state), '[%s]' % ', '.join(map(str, x)))

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
    print('> inv [%s]' % ', '.join(map(str, xd[i])))
    xd_vector = yarp.DVector(xd[i])
    qd_vector = yarp.DVector()

    if cc.inv(xd_vector, qd_vector):
        print('< [%s]' % ', '.join(map(str, qd_vector)))
    else:
        print('< [fail]')
        continue

    print('> movj [%s]' % ', '.join(map(str, xd[i])))
    xd_vector = yarp.DVector(xd[i])

    if cc.movj(xd_vector):
        print('< [ok]')
        print('< [wait...]')
        cc.wait()
    else:
        print('< [fail]')

print('bye!')

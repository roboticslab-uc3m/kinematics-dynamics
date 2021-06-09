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

cartesianControl = kd.viewICartesianControl(dd)

print('> stat')
x = yarp.DVector()
ret, state, ts = cartesianControl.stat(x)
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

    if cartesianControl.inv(xd_vector, qd_vector):
        print('< [%s]' % ', '.join(map(str, qd_vector)))
    else:
        print('< [fail]')
        continue

    print('> movj [%s]' % ', '.join(map(str, xd[i])))
    xd_vector = yarp.DVector(xd[i])

    if cartesianControl.movj(xd_vector):
        print('< [ok]')
        print('< [wait...]')
        cartesianControl.wait()
    else:
        print('< [fail]')

print('bye!')

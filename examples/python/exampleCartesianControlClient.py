#! /usr/bin/env python

import yarp
import kinematics_dynamics

yarp.Network.init()

if yarp.Network.checkNetwork() != True:
    print '[error] Please try running yarp server'
    raise SystemExit

options = yarp.Property()
options.put('device','CartesianControlClient')
options.put('cartesianRemote','/teoSim/rightArm/CartesianControl')
options.put('cartesianLocal','/cartesianControlExample')
options.put('transform', 1)
dd = yarp.PolyDriver(options)  # calls open -> connects

if not dd.isValid():
    print 'Cannot open the device!'
    raise SystemExit

cartesianControl = kinematics_dynamics.viewICartesianControl(dd)  # view the actual interface

print '> stat'
x = yarp.DVector()
ret, state, ts = cartesianControl.stat(x)
print '<', yarp.Vocab.decode(state), '[%s]' % ', '.join(map(str, x))

xd = [0,0,0,0,0,0,0]

xd[0] = [0.389496, -0.34692, 0.16769, 1.0, 0.0, 0.0, 0.0]
xd[1] = [0.5, -0.34692, 0.16769, 1.0, 0.0, 0.0, 0.0]
xd[2] = [0.5, -0.4, 0.16769, 1.0, 0.0, 0.0, 0.0]
xd[3] = [0.5, -0.4, 0.16769, 0.0, 1.0, 0.0, -12.0]
xd[4] = [0.5, -0.4, 0.16769, 1.0, 0.0, 0.0, -50.0]
xd[5] = [0.389496, -0.34692, 0.16769, 1.0, 0.0, 0.0, 0.0]
xd[6] = [0.0, -0.34692, -0.221806, 0.0, 1.0, 0.0, 90.0]

for i in range(len(xd)):
    print '-- movement '+str(i+1)+':'
    print '> inv [%s]' % ', '.join(map(str, xd[i]))
    xd_vector = yarp.DVector(xd[i])
    qd_vector = yarp.DVector()
    if cartesianControl.inv(xd_vector,qd_vector):
        print '< [%s]' % ', '.join(map(str, qd_vector))
    else:
        print '< [fail]'
        continue
    
    print '> movj [%s]' % ', '.join(map(str, xd[i]))
    xd_vector = yarp.DVector(xd[i])
    if cartesianControl.movj(xd_vector):
        print '< [ok]'
        print '< [wait...]'
        cartesianControl.wait()
    else:
        print '< [fail]'
    
    

print 'bye!'

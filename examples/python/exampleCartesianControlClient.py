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
stat = cartesianControl.stat(x)
print '<',yarp.Vocab.decode(stat),'[%s]' % ', '.join(map(str, x))

xd = [0.389496, -0.34692, 0.16769, 1.0, 0.0, 0.0, 20.0]

print '> inv [%s]' % ', '.join(map(str, xd))
xd_vector = yarp.DVector(xd)
qd_vector = yarp.DVector()
if cartesianControl.inv(xd_vector,qd_vector):
    print '< [%s]' % ', '.join(map(str, qd_vector))
else:
    print '< [fail]'

print '> movj [%s]' % ', '.join(map(str, xd))
xd_vector = yarp.DVector(xd)
if cartesianControl.movj(xd_vector):
    print '< [ok]'
else:
    print '< [fail]'

print 'delay(1)'
yarp.Time.delay(1)

print 'bye!'

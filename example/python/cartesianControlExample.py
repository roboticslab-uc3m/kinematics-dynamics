#! /usr/bin/env python

import yarp
import teo

yarp.Network.init()

if yarp.Network.checkNetwork() != True:
    print '[error] Please try running yarp server'
    quit()

options = yarp.Property()
options.put('device','CartesianControlClient')
options.put('cartesianRemote','/teoSim/rightArm/CartesianControlServer')
options.put('cartesianLocal','/cartesianControlExample')
dd = yarp.PolyDriver(options)  # calls open -> connects

cartesianControl = teo.viewICartesianControl(dd)  # view the actual interface

print '> stat'
x = yarp.DVector()
stat = cartesianControl.stat(x)
print '<',yarp.Vocab.decode(stat),'[%s]' % ', '.join(map(str, x))

xd = [0,-0.346927436108, -0.221801094416,0,1,0,90]

print '> inv [%s]' % ', '.join(map(str, xd))
xd_vector = yarp.DVector(xd)
qd_vector = yarp.DVector()
cartesianControl.inv(xd_vector,qd_vector)
print '< [%s]' % ', '.join(map(str, qd_vector))

print '> movj [%s]' % ', '.join(map(str, xd))
xd_vector = yarp.DVector(xd)
if cartesianControl.movj(xd_vector):
    print '< [ok]'
else:
    print '< [fail]'

print 'delay(1)'
yarp.Time.delay(1)

print 'bye!'


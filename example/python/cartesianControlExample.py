#! /usr/bin/env python

import yarp
import teo

yarp.Network.init()

if yarp.Network.checkNetwork() != True:
    print "[error] Please try running yarp server"
    quit()

options = yarp.Property()
options.put('device','CartesianControlClient')
options.put('cartesianRemote','/teoSim/rightArm/CartesianControlServer')
options.put('cartesianLocal','/cartesianControlExample')
dd = yarp.PolyDriver(options)  # calls open -> connects

cartesianControl = teo.viewICartesianControl(dd)  # view the actual interface

print "stat"
#a = (int)
#b = yarp.DVector()
#cartesianControl.stat(a,b)

print "delay(5)"
yarp.Time.delay(5)

#print ""
#cartesianControl.


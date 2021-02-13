#! /usr/bin/env python

import yarp
import kinematics_dynamics

#-- Locate the file with the kinematic chain DH parameters

rf = yarp.ResourceFinder()
rf.setVerbose(False)
rf.setDefaultContext("testKdlSolverFromFile")
rf.setDefaultConfigFile("testKdlSolverFromFile.ini")
kinematicsFileFullPath = rf.findFileByName("testKdlSolverFromFile.ini")

#-- Load the solver device with this configuration, view the interface

solverOptions = yarp.Property()
solverOptions.fromConfigFile(kinematicsFileFullPath)
solverOptions.put("device","KdlSolver")
solverOptions.fromString("(mins (-70 -15 -10 -100 -90 -100)) (maxs (45 70 75 10 90 10))", False)

solverDevice = yarp.PolyDriver(solverOptions)

if not solverDevice.isValid():
    print("Cannot open the device!")
    raise SystemExit

cartesianSolver = kinematics_dynamics.viewICartesianSolver(solverDevice) # view the actual interface

##-- Illustration of forward kinematics, for different joint-space positions

print('--- Joint space configuration 1: expect Cartesian space position 0, 0.34692, -0.221806')

q = [0,0,0,0,0,0,0]
q_vector = yarp.DVector(q)
x_vector = yarp.DVector()

cartesianSolver.fwdKin(q_vector,x_vector);

print('> fwdKin [%s]' % ', '.join(map(str, q_vector)))
print('< [%s]' % ', '.join(map(str, x_vector)))

print('--- Joint space configuration 2: expect Cartesian space position 0.718506, 0.34692, 0.4967')

q = [-90,0,0,0,0,0,0]
q_vector = yarp.DVector(q)
x_vector = yarp.DVector()

cartesianSolver.fwdKin(q_vector,x_vector);

print('> fwdKin [%s]' % ', '.join(map(str, q_vector)))
print('< [%s]' % ', '.join(map(str, x_vector)))

print('bye!')

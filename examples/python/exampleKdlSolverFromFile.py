#! /usr/bin/env python

import yarp
import kinematics_dynamics

rf = yarp.ResourceFinder()
rf.setVerbose(False)
rf.setDefaultContext("testKdlSolverFromFile")
rf.setDefaultConfigFile("testKdlSolverFromFile.ini")
kinematicsFileFullPath = rf.findFileByName("testKdlSolverFromFile.ini")

solverOptions = yarp.Property()
solverOptions.fromConfigFile(kinematicsFileFullPath)
solverOptions.put("device","KdlSolver")
solverOptions.fromString("(mins (-70 -15 -10 -100 -90 -100)) (maxs (45 70 75 10 90 10))", False)

solverDevice = yarp.PolyDriver(solverOptions)

if not solverDevice.isValid():
    print("Cannot open the device!")
    raise SystemExit


cartesianSolver = kinematics_dynamics.viewICartesianSolver(solverDevice) # view the actual interface

# Configuration 1

q = [0,0,0,0,0,0,0]
q_vector = yarp.DVector(q)
x_vector = yarp.DVector()

cartesianSolver.fwdKin(q_vector,x_vector);

print('--- Configuration 1: expect 0, 0.34692, -0.221806')
print('> fwdKin [%s]' % ', '.join(map(str, q_vector)))
print('< [%s]' % ', '.join(map(str, x_vector)))

# Configuration 2

q = [-90,0,0,0,0,0,0]
q_vector = yarp.DVector(q)
x_vector = yarp.DVector()

cartesianSolver.fwdKin(q_vector,x_vector);

print('--- Configuration 2: expect 0.718506, 0.34692, 0.4967')
print('> fwdKin [%s]' % ', '.join(map(str, q_vector)))
print('< [%s]' % ', '.join(map(str, x_vector)))

print('bye!')

#!/usr/bin/env python3

import time
import numpy as np
import yarp
import PyKDL as kdl
import PySTS as sts
import threading
from motion_3 import *

# Placeholder imports for custom classes
# from your_module import PoeExpression, MatrixExponential, ScrewTheoryIkProblemBuilder, ConfigurationSelectorLeastOverallAngularDisplacementFactory, TrajectoryThread

DEFAULT_REMOTE_PORT = "/ur16esim"
DEFAULT_TRAJ_DURATION = 10.0
DEFAULT_TRAJ_MAX_VEL = 0.05
DEFAULT_PERIOD_MS = 50.0

def make_ur16e_kinematics():
    H_ST_0 = kdl.Frame(kdl.Vector(0.838, 0.364, 0.061))
    poe = sts.PoeExpression(H_ST_0)

    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 0, 1), kdl.Vector(0, 0, 0.181)))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 1, 0), kdl.Vector(0, 0, 0.181)))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 1, 0), kdl.Vector(0.478, 0, 0.181)))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 1, 0), kdl.Vector(0.838, 0.174, 0.181)))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 0, -1), kdl.Vector(0.838, 0.174, 0.061)))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.motion.ROTATION, kdl.Vector(0, 1, 0), kdl.Vector(0.838, 0.174, 0.061)))

    return poe

if __name__ == "__main__":
    yarp.Network.init()

    if not yarp.Network.checkNetwork():
        print("Please start a yarp name server first")
        exit(1)

    # Argument parsing (simplified)
    remote = DEFAULT_REMOTE_PORT
    traj_duration = DEFAULT_TRAJ_DURATION
    traj_max_vel = DEFAULT_TRAJ_MAX_VEL
    period_ms = DEFAULT_PERIOD_MS

    # Device options
    joint_device_options = yarp.Property()
    joint_device_options.put("device", "remote_controlboard")
    joint_device_options.put("remote", remote)
    joint_device_options.put("local", "/screwTheoryTrajectoryExample" + remote)

    joint_device = yarp.PolyDriver(joint_device_options)

    if not joint_device.isValid():
        print("Joint device not available")
        exit(1)

    iEncoders = joint_device.viewIEncoders()
    iControlLimits = joint_device.viewIControlLimits()
    iControlMode = joint_device.viewIControlMode()
    iPositionDirect = joint_device.viewIPositionDirect()

    if not (iEncoders and iControlLimits and iControlMode and iPositionDirect):
        print("Problems acquiring joint interfaces")
        exit(1)

    axes = iEncoders.getAxes()
    q = yarp.DVector(axes)

    while not iEncoders.getEncoders(q):
        time.sleep(0.1)

    poe = make_ur16e_kinematics()
    axes = poe.size()  # For real TEO (7 joints, 6 motor axes)

    jnt_array = kdl.JntArray(axes)
    for i in range(axes):
        jnt_array[i] = np.deg2rad(q[i])

    H_base_start = kdl.Frame()
    if not poe.evaluate(jnt_array, H_base_start):
        print("FK error")
        exit(1)

    builder = sts.ScrewTheoryIkProblemBuilder(poe)
    ik_problem = builder.build()

    if ik_problem is None:
        print("Unable to solve IK")
        exit(1)

    q_min = kdl.JntArray(axes)
    q_max = kdl.JntArray(axes)

    for i in range(axes):
        min, max = yarp.DVector(1), yarp.DVector(1)
        iControlLimits.getLimits(i, min, max)
        q_min[i] = math.radians(min[0])
        q_max[i] = math.radians(max[0])

    ik_config = sts.ConfigurationSelectorLeastOverallAngularDisplacement(q_min, q_max)

    H_base_end = kdl.Frame(H_base_start)
    H_base_end.p = H_base_end.p + kdl.Vector(-0.3, 0.0, 0.0)

    path = PathLine(H_base_start, H_base_end)
    profile = VelocityProfileRectangular(traj_max_vel)
    trajectory = TrajectorySegment(path, profile, traj_duration)

    if not iControlMode.setControlModes(yarp.IVector(axes, yarp.encode('posd'))):
        print("Unable to change mode")
        exit(1)

    should_stop = False

    def run():
        start_time = time.time()

        while not should_stop:
            movement_time = time.time() - start_time
            H_S_T = trajectory.position(movement_time)
            solutions, reachability = ik_problem.solve(H_S_T)

            # if not ik_config.configure(solutions, reachability):
            #     print("IK configuration failed")
            #     return

            # q = kdl.JntArray(axes)
            # q_deg = yarp.DVector(axes)

            # if not iEncoders.getEncoders(q_deg):
            #     print("getEncoders() failed")
            #     return

            # for i in range(q.rows()):
            #     q[i] = math.radians(q_deg[i])
            # print(q)
            # if not ik_config.findOptimalConfiguration(q):
            #     print("findOptimalConfiguration() failed")
            #     return

            # solution = ik_config.retrievePose()
            solution = solutions[0]

            refs = yarp.DVector([math.degrees(v) for v in solution])
            print(f"IK -> {list(refs)}")

            if not iPositionDirect.setPositions(refs):
                print("setPositions() failed")

    t = threading.Thread(target=run)
    t.start()

    time.sleep(traj_duration)
    should_stop = True
    t.join()

    joint_device.close()
    yarp.Network.fini()
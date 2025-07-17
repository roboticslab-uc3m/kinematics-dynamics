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

    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 0, 1], [0, 0, 0.181]))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 1, 0], [0, 0, 0.181]))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 1, 0], [0.478, 0, 0.181]))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 1, 0], [0.838, 0.174, 0.181]))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 0, -1], [0.838, 0.174, 0.061]))
    poe.append(sts.MatrixExponential(sts.MatrixExponential.ROTATION, [0, 1, 0], [0.838, 0.174, 0.061]))

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
        q_min[i] = min[0]
        q_max[i] = max[0]

    conf_factory = sts.ConfigurationSelectorLeastOverallAngularDisplacementFactory(q_min, q_max)
    ik_config = conf_factory.create()

    H_base_end = kdl.Frame(H_base_start)
    H_base_end.p = H_base_end.p + kdl.Vector(-0.3, 0.0, 0.0)

    interp = RotationalInterpolation_SingleAxis()
    path = PathLine(H_base_start, H_base_end, interp, 0.1, False)
    profile = VelocityProfileRect(traj_max_vel, 0.2)
    trajectory = TrajectorySegment(path, profile, traj_duration, False)

    if not iControlMode.setControlModes(yarp.IVector(axes, yarp.encode('posd'))):
        print("Unable to change mode")
        exit(1)

    def run():
        movement_time = yarp.Time.now() - self.start_time           
        H_S_T: kdl.Frame = self.trajectory.Pos(movement_time)       

        self.print_cartesian_coordinates(H_S_T, movement_time)     

        solutions: List[kdl.JntArray] = []
        reachability = self.ik_problem.solve(H_S_T, solutions)    

        q = kdl.JntArray(self.axes)                               

        encoders = yarp.Vector(self.axes)
        if not self.iEncoders.getEncoders(encoders.data()):
            yarp.log.error("getEncoders() failed")
            return

        for i in range(q.rows()):
            q[i] = math.radians(encoders[i])                  

        solution: kdl.JntArray = solutions[0]

        refs = [math.degrees(v) for v in solution]               
        yarp.log.info(f"IK -> {refs}")

        if not self.iPosDirect.setPositions(refs):
            yarp.log.error("setPositions() failed")

    t = threading.Thread(target=run)
    # t.start()

    # # traj_thread = TrajectoryThread(iEncoders, iPositionDirect, ik_problem, ik_config, trajectory, period_ms)

    # time.sleep(traj_duration)
    # t.join()

    joint_device.close()
    yarp.Network.fini()

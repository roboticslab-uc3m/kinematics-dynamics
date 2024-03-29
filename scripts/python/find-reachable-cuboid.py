import argparse
import math
import yarp
import roboticslab_kinematics_dynamics as kd

# https://stackoverflow.com/a/17954769/10404307
from contextlib import contextmanager
import sys, os

@contextmanager
def stderr_redirected(to=os.devnull):
    fd = sys.stderr.fileno()

    def _redirect_stderr(to):
        sys.stderr.close()
        os.dup2(to.fileno(), fd)
        sys.stderr = os.fdopen(fd, 'w')

    with os.fdopen(os.dup(fd), 'w') as old_stderr:
        with open(to, 'w') as file:
            _redirect_stderr(to=file)
        try:
            yield
        finally:
            _redirect_stderr(to=old_stderr)

parser = argparse.ArgumentParser(description='Obtain the largest cuboid-shaped reachable space of a manipulator around an initial pose')
parser.add_argument('--step', default=0.01, type=float, help='IK step [m]')
parser.add_argument('--minX', default=-math.inf, type=float, help='lower X axis bound [m]')
parser.add_argument('--maxX', default=math.inf, type=float, help='upper X axis bound [m]')
parser.add_argument('--minY', default=-math.inf, type=float, help='lower Y axis bound [m]')
parser.add_argument('--maxY', default=math.inf, type=float, help='upper Y axis bound [m]')
parser.add_argument('--minZ', default=-math.inf, type=float, help='lower Z axis bound [m]')
parser.add_argument('--maxZ', default=math.inf, type=float, help='upper Z axis bound [m]')
parser.add_argument('--kinematics', required=True, type=str, help='kinematics.ini file')
parser.add_argument('--eps', default=10e-3, type=float, help='epsilon value for IK float operations')
parser.add_argument('--limitsMin', required=True, nargs='+', type=float, help='lower joint limits [deg]')
parser.add_argument('--limitsMax', required=True, nargs='+', type=float, help='upper joint limits [deg]')
parser.add_argument('--initialJointPose', required=True, nargs='+', type=float, help='initial joint pose [deg]')
args = parser.parse_args()

if not args.minX < args.maxX:
    raise ValueError('--minX must be less than --maxX')

if not args.minY < args.maxY:
    raise ValueError('--minY must be less than --maxY')

if not args.minZ < args.maxZ:
    raise ValueError('--minZ must be less than --maxZ')

if len(args.limitsMin) != len(args.limitsMax) or len(args.initialJointPose) != len(args.limitsMin):
    raise ValueError('--limitsMin, --limitsMax and --initialJoinPose must have the same length')

for i in range(len(args.limitsMin)):
    if args.limitsMin[i] > args.limitsMax[i]:
        raise ValueError('--limitsMin[%d] must not be greater than --limitsMax[%d]: %f > %f' % (i, i, args.limitsMin[i], args.limitsMax[i]))

# reference limits for TEO's left arm
# args.limitsMin = [-96.8, -23.9, -51.6, -101.1, -101.3, -113.3]
# args.limitsMax = [113.2, 76.5, 84.1, 96.8, 76.4, 61.3]

def check_limits(values):
    for i, value in enumerate(values):
        if not args.limitsMin[i] <= value <= args.limitsMax[i]:
            return False

    return True

# reference initial joint pose for TEO's left (or right) arm
# args.initialJointPose = [-45, 0, 0, -30, 0, -15]

options = yarp.Property()
options.put('device', 'KdlSolver')
options.put('kinematics', args.kinematics)
options.put('ikPos', 'lma')
options.put('weights', '1 1 1 0 0 0') # dismiss orientation
options.put('epsPos', args.eps)

device = yarp.PolyDriver(options)

if not device.isValid():
    print('device not available')
    quit()

solver = kd.viewICartesianSolver(device)

print('Initial joint pose:', args.initialJointPose)
initial_joint_pose = yarp.DVector(args.initialJointPose)

initial_cart_pose = yarp.DVector()
solver.fwdKin(initial_joint_pose, initial_cart_pose)

print('Initial cartesian pose:', list(initial_cart_pose))

desired_cart_pose = yarp.DVector(initial_cart_pose)
desired_joint_pose = yarp.DVector(6)

# algorithm:
# 1. find the cuboid that encapsulates the reachable space around an initial pose
# 2. perform IK for all points in said cuboid
# 3. find the largest inner cuboid containing points with a valid IK

# [[minX, maxX], [minY, maxY], [minZ, maxZ]]
outer_bounding_box = [[value, value] for value in initial_cart_pose][:3]

with stderr_redirected(): # dismiss a few irrelevant KDL errors
    for axis in range(3): # [X, Y, Z]
        for direction in range(2): # [MIN, MAX]
            while True:
                diff = args.step * (1 if direction == 1 else -1)
                desired_cart_pose[axis] += diff

                if (
                        axis == 0 and not (args.minX < desired_cart_pose[axis] < args.maxX) or
                        axis == 1 and not (args.minY < desired_cart_pose[axis] < args.maxY) or
                        axis == 2 and not (args.minZ < desired_cart_pose[axis] < args.maxZ) or
                        not solver.invKin(desired_cart_pose, initial_joint_pose, desired_joint_pose) or
                        not check_limits(desired_joint_pose)
                    ):
                    outer_bounding_box[axis][direction] = desired_cart_pose[axis] - diff
                    desired_cart_pose = yarp.DVector(initial_cart_pose)
                    break

print('Outer boundaries (X, Y, Z) [m]:', list(outer_bounding_box))

dims = [boundary[1] - boundary[0] for boundary in outer_bounding_box]
print('Dimensions [m]:', dims)

dims_n = [int(dim / args.step) for dim in dims]
print('Dimensions [#]:', dims_n)

ik_solutions = [[[False for z in range(dims_n[2])] for y in range(dims_n[1])] for x in range(dims_n[0])]
total_problems = math.prod(dims_n)

print('IK problems:', total_problems)

input('Press enter to start solving IK...')
progress = 0
successes = 0

with stderr_redirected(): # KDL errors are way too verbose here
    for x in range(dims_n[0]):
        for y in range(dims_n[1]):
            for z in range(dims_n[2]):
                desired_cart_pose[:3] = [outer_bounding_box[axis][0] + args.step * value for axis, value in enumerate([x, y, z])]

                if (
                    solver.invKin(desired_cart_pose, initial_joint_pose, desired_joint_pose) and
                    check_limits(desired_joint_pose)
                ):
                    ik_solutions[x][y][z] = True
                    successes += 1

                progress += 1

                if progress % 10000 == 0:
                    print('Progress: %d/%d' % (progress, total_problems))

print('Successful IK solutions: %d out of %d' % (successes, total_problems))

initial_cart_indices = [int((value - outer_bounding_box[axis][0]) / args.step) for axis, value in enumerate(initial_cart_pose[:3])]
print('Initial cartesian pose indices:', initial_cart_indices)

input('Press enter to start finding largest inner cuboid...')

global_volume = 1
largest_cuboid_indices = tuple((value, value) for value in initial_cart_indices)

# https://stackoverflow.com/a/9925015/10404307

def find_largest_segment(binary_vector):
    global_length = 1
    largest_segment_indices = largest_cuboid_indices[2:][0] # mind the [0]!

    for z0 in range(0, initial_cart_indices[2]):
        scalar = True

        for z in range(z0, dims_n[2]):
            scalar &= binary_vector[z]

            if z < initial_cart_indices[2]:
                continue

            if not scalar:
                break

            local_segment_indices = (z0, z)
            local_length = z - z0 + 1

            if local_length > global_length:
                largest_segment_indices = local_segment_indices
                global_length = local_length

    return largest_segment_indices

def find_largest_area(binary_array):
    global_area = 1
    largest_rectangle_indices = largest_cuboid_indices[1:]

    for y0 in range(0, initial_cart_indices[1]):
        array1d = [True for z in range(dims_n[2])]

        for y in range(y0, dims_n[1]):
            for z in range(dims_n[2]):
                array1d[z] &= binary_array[y][z]

            if y < initial_cart_indices[1]:
                continue

            z_range = find_largest_segment(array1d)
            local_rectangle_indices = ((y0, y), z_range)
            local_area = math.prod(map(lambda v: v[1] - v[0] + 1, local_rectangle_indices))

            if local_area > global_area:
                largest_rectangle_indices = local_rectangle_indices
                global_area = local_area

    return largest_rectangle_indices

for x0 in range(0, initial_cart_indices[0]):
    print('Progress: %d%%' % int(100 * x0 / initial_cart_indices[0]))
    array2d = [[True for z in range(dims_n[2])] for y in range(dims_n[1])]

    for x in range(x0, dims_n[0]):
        for y in range(dims_n[1]):
            for z in range(dims_n[2]):
                array2d[y][z] &= ik_solutions[x][y][z]

        if x < initial_cart_indices[0]:
            continue

        y_range, z_range = find_largest_area(array2d)
        local_cuboid_indices = ((x0, x), y_range, z_range)
        local_volume = math.prod(map(lambda v: v[1] - v[0] + 1, local_cuboid_indices))

        if local_volume > global_volume:
            largest_cuboid_indices = local_cuboid_indices
            global_volume = local_volume

print('Largest cuboid indices:', largest_cuboid_indices)

inner_bounding_box = [[outer_bounding_box[axis][0] + value * args.step for value in range] for axis, range in enumerate(largest_cuboid_indices)]
print('Inner boundaries:', inner_bounding_box)
print('Dimensions [m]:', [boundary[1] - boundary[0] for boundary in inner_bounding_box])

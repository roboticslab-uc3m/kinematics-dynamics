#!/usr/bin/env python

# See README.md

from PyKDL import *
from math import pi
import csv

lengthsFileName = '/home/yo/repos/teo-developer-manual/csv/lengths.csv'
dhFileName = '/home/yo/repos/teo-developer-manual/csv/dh-root-rightArm.csv'

chain = Chain()

def degToRad(deg):
    return deg*pi/180.0

#-- initPoss
# Small hack: first replace two digits, e.g. else can replace q1 before q13
twoDigitJoints = {'q'+str(i):'0.0' for i in range(10,28)} # dof
oneDigitjoints = {'q'+str(i):'0.0' for i in range(1,10)} # dof

#-- lengths
lengthsFile = open(lengthsFileName, 'r')
reader = csv.reader(lengthsFile, delimiter=',')
next(reader, None)  # skip the header
lengths = {row[0]:str(float(row[1])/1000.0) for row in reader} # [mm] to [m]
print('lengths: ',lengths)

#-- dh params
dhFile = open(dhFileName, 'r')
reader = csv.reader(dhFile, delimiter=',')

#-- populate chain
next(reader, None)  # skip the header
for row in reader:
    print('row: ',row)

    linkOffset = row[1]
    for first, second in twoDigitJoints.items():
        linkOffset = str(linkOffset).replace(first, second)
    for first, second in oneDigitjoints.items():
        linkOffset = str(linkOffset).replace(first, second)
    linkOffset = eval(linkOffset)

    linkD = row[2]
    for first, second in twoDigitJoints.items():
        linkD = str(linkD).replace(first, second)
    for first, second in oneDigitjoints.items():
        linkD = str(linkD).replace(first, second)
    for first, second in lengths.items():
        linkD = str(linkD).replace(first, second)
    linkD = eval(linkD)

    linkA = row[3]
    for first, second in lengths.items():
        linkA = str(linkA).replace(first, second)
    linkA = eval(linkA)

    linkAlpha = eval(row[4])

    print('* linkOffset ', linkOffset)
    print('* linkD ', linkD)
    print('* linkA ', linkA)
    print('* linkAlpha ', linkAlpha)
    s = Segment(Joint(Joint.RotZ),
                Frame().DH(linkA,degToRad(linkAlpha),linkD,degToRad(linkOffset)))
    chain.addSegment(s)

fksolverpos = ChainFkSolverPos_recursive(chain)
q = JntArray(chain.getNrOfJoints())
x = Frame()
fksolverpos.JntToCart(q,x)

print(x)

print("({:.6f} {:.6f} {:.6f} {:.6f}\
    {:.6f} {:.6f} {:.6f} {:.6f}\
    {:.6f} {:.6f} {:.6f} {:.6f}\
    0 0 0 1)"
    .format(
    x.M[0,0], x.M[0,1], x.M[0,2], x.p[0],
    x.M[1,0], x.M[1,1], x.M[1,2], x.p[1],
    x.M[2,0], x.M[2,1], x.M[2,2], x.p[2]))

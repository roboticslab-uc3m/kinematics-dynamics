#!/usr/bin/env python

# See README.md

from __future__ import print_function

from PyKDL import *
from math import pi
import csv

import os
home = os.environ['HOME']

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--lengthsFileName", default=home+"/repos/teo-developer-manual/csv/lengths.csv")
parser.add_argument("--dhFileName", default=home+"/repos/teo-developer-manual/csv/dh-root-rightArm.csv")
args = parser.parse_args()

chain = Chain()

def degToRad(deg):
    return deg*pi/180.0

#-- initPoss
# Small hack: first replace two digits, e.g. else can replace q1 before q13
twoDigitJoints = {'q'+str(i):'0.0' for i in range(10,28)} # dof
oneDigitjoints = {'q'+str(i):'0.0' for i in range(1,10)} # dof

#-- lengths
lengthsFile = open(args.lengthsFileName, 'r')
reader = csv.reader(lengthsFile, delimiter=',')
next(reader, None)  # skip the header
lengths = {row[0]:str(float(row[1])/1000.0) for row in reader} # [mm] to [m]
print('lengths: ',lengths)

#-- dh params
dhFile = open(args.dhFileName, 'r')
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

#print(x)

def s(inputValue):
    tmp = ('%.15f' % inputValue).rstrip('0').rstrip('.')
    if tmp == "-0":
        return "0"
    else:
        return tmp

print("(", s(x.M[0,0]), s(x.M[0,1]), s(x.M[0,2]), s(x.p[0]), "  ",
           s(x.M[1,0]), s(x.M[1,1]), s(x.M[1,2]), s(x.p[1]), "  ",
           s(x.M[2,0]), s(x.M[2,1]), s(x.M[2,2]), s(x.p[2]), "  ",
           "0 0 0 1 )")

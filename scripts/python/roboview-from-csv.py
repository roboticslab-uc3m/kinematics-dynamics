# See README.md

from PyKDL import *
from math import pi
import csv

import os
home = os.environ['HOME']

lengthsFileName = home+'/repos/teo-developer-manual/csv/lengths.csv'
dhFileName = home+'/repos/teo-developer-manual/csv/dh-rightArm.csv'

segments = [] # required for roboview

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

#-- populate segments
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
    segments.append(s)


#-- populate limits (hard-coded for now)
limits_min = [-180.0] * len(segments)
limits_max = [180.0] * len(segments)

limits_min = [degToRad(v) for v in limits_min]
limits_max = [degToRad(v) for v in limits_max]

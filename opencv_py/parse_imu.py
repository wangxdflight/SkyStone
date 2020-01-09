import sys
import string
import math

data=[];
data2=[];
filepath = sys.argv[1];

with open(filepath) as fp:
    line = fp.readline()
    #print(line)
    while line:
        line = fp.readline();
        #print(line)
        if (("StandardTrackingWheelLocalizer" in line) or ("using IMU" in line)) and ("non-IMU" in line):
            #print(line)
            t = line.split('IMU heading')
            #print(t)
            t1 = t[1].split(' ')
            data.append(t1[1])
            t = line.split('non-IMU heading:')
            #print(t)
            t1 = t[1].split(' ')
            data2.append(t1[1])

            #input("Press Enter to continue...")

    for i in range(len(data)):
        d1 = math.degrees(float(data[i].strip()));
        d2 = math.degrees(float(data2[i].strip()))
        if d2 > 180:
            d2 = d2 - 360
        print(d1, d2);


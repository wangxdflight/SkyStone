import sys
import string

data=[];
data_x=[];
data_y=[];
data_h=[];
data_v_err=[];
data_v_target=[];
data_v_actual=[];
filepath = sys.argv[1];
with open(filepath) as fp:
    line = fp.readline()
    while line:
        line = fp.readline();
        if ("BaseClass" in line) and ("Error" in line):
            #t = line.strip();
            t = line;
            if ("xError" in t):
                t1 = t.split('xError')
                #print(t1)
                data_x.append(t1[1])
            if ("yError" in t):
                t1 = t.split('yError')
                data_y.append(t1[1])
            if ("headingError" in t):
                t1 = t.split('headingError')
                data_h.append(t1[1])
        if ("DriveVelocityPIDTuner: error 0" in line):
            t = line;
            t1 = t.split('error 0');
            data_v_err.append(t1[1])
        if ("DriveVelocityPIDTuner: targetVelocity" in line):
            t = line;
            t1 = t.split('targetVelocity')
            data_v_target.append(t1[1])
        if ("DriveVelocityPIDTuner: velocity 0" in line):
            t = line;
            t1 = t.split('velocity 0');
            data_v_actual.append(t1[1])
    for i in range(len(data_x)):
        print(data_x[i].strip(), " ", data_y[i].strip(), " ", data_h[i].strip());
    for i in range(len(data_v_err)):
        print(data_v_err[i].strip(), " ", data_v_target[i].strip(), " ", data_v_actual[i].strip());

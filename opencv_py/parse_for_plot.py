import sys
import string
import math
from matplotlib import pyplot as plt
import numpy as nm

data=[];
data_x=[];  # xError in follower;
data_x_raw=[];  # current Pose
data_y=[];
data_y_raw=[];
data_h=[];   # headingError
data_h_raw=[];
data_v_err=[];
data_v_target=[];
data_v_actual=[];
data_power=[];
max_power=0;
max_x_err=0;
max_y_err=0;
max_heading_err=0;
max_v=0;
p_name='noname';
print_summary=0;
filepath = sys.argv[1];
arg_c = len(sys.argv);
if arg_c>=3:
    print_summary = 1;

with open(filepath) as fp:
    line = fp.readline()
    #print(line)
    while line:
        line = fp.readline();
        #print(line)
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("update: x" in line):
            t1 = line.split("update: x");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            t = t3[0];
            data_x_raw.append(float(t));

        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("Base: y " in line):
            t1 = line.split("Base: y");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            t = t3[0];
            data_y_raw.append(float(t));
            #print("y: ", t);
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("Base: heading " in line):
            t1 = line.split("Base: heading");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            t = t3[0];
            data_h_raw.append(math.degrees(float(t)));
            #print("y: ", t);

        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("Error" in line):
            #t = line.strip();
            if ("xError" in line):
                t1 = line.split('xError')
                #print(t1)
                t2 = t1[1]
                t = float(t2)
                data_x.append(t)
                if t > max_x_err:
                    max_x_err = t;
            if ("yError" in line):
                t1 = line.split('yError')
                t2 = t1[1]
                t = float(t2)
                data_y.append(t)
                if t > max_y_err:
                    max_y_err = t;
            if ("headingError" in line):
                t1 = line.split('headingError')
                t2 = t1[1]
                t = math.degrees(float(t2));
                data_h.append(t)
                if t > max_heading_err:
                    max_heading_err = t;
        #####################################
        if ("DriveVelocityPIDTuner: error 0" in line):
            #print(t);
            t1 = line.split('error 0');
            data_v_err.append(float(t1[1]))
        if ("DriveVelocityPIDTuner: targetVelocity" in line):
            t1 = line.split('targetVelocity')
            data_v_target.append(float(t1[1]))
        if ("DriveVelocityPIDTuner: velocity 0" in line):
            t1 = line.split('velocity 0');
            t2 = t1[1];
            data_v_actual.append(float(t2))
            t = float(t2.strip());
            t = abs(t)
            if t > max_v:
                max_v = t;
        #############################################
        if ("setMotorPowers" in line) and ("leftFront" in line):
            #print(line)
            t = line.split('setMotorPowers');
            t1 = t[1].strip().split(' ');
            #print(t1)
            t2 = t1[1]
            data_power.append(float(t2))
            t = float(t2)
            if t>max_power:
                max_power=t;
        ###########################################
        if ("RobotCore" in line) and ("START - OPMODE" in line):
            t = line.split('OPMODE');
            t1 = t[1].strip().split(' ')
            p_name=t1[0]

    for i in range(len(data_x)):
        print(data_x[i], " ", data_y[i], " ", data_h[i], " ", data_x_raw[i], " ", data_y_raw[i], " ", data_h_raw[i]);

    for i in range(len(data_v_err)):
        print(data_v_err[i].strip(), " ", data_v_target[i].strip(), " ", data_v_actual[i].strip());

if print_summary != 0:
    plt.interactive(False)

    plt.style.use('ggplot')
    plt.plot(data_x, label="xError");
    plt.plot(data_y, label="yError");
    plt.plot(data_h, label="headingError");
    plt.xlabel('time');
    plt.ylabel('inches for x, y, degrees for heading');
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
               ncol=2, mode="expand", borderaxespad=0.)
    plt.figure();

    plt.plot(nm.add(data_x, data_x_raw), label="target X");
    plt.plot(data_x_raw, label="actual X")
    plt.xlabel('time');
    plt.ylabel('inches');
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
               ncol=2, mode="expand", borderaxespad=0.)
    plt.figure();

    plt.plot(nm.add(data_y, data_y_raw), label="target Y");
    plt.plot(data_y_raw, label="actual Y")
    plt.xlabel('time');
    plt.ylabel('inches');
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
               ncol=2, mode="expand", borderaxespad=0.)
    plt.figure();

    plt.plot(nm.add(data_h, data_h_raw), label="target heading");
    plt.plot(data_h_raw, label="actual heading")
    plt.xlabel('time');
    plt.ylabel('degrees');
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
               ncol=2, mode="expand", borderaxespad=0.)

    print("===============summary==========================")
    print("data_x, data_y, data_h, data_x_raw, data_y_raw, data_heading_raw");
    print("data_v, data_v_target, data_v_actual");
    print("program : ", p_name)
    print("max power to wheel: ", max_power)
    print("max_x_err (inches): ", max_x_err)
    print("max_y_err (inches): ", max_y_err)
    print("max_heading_err (degrees) ", max_heading_err)
    print("max_velocity : ", max_v)


    plt.show();
    plt.close()

import sys
import string
import math

data=[];
data_x=[];
data_y=[];
data_h=[];
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
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("Error" in line):
            #t = line.strip();
            if ("xError" in line):
                t1 = line.split('xError')
                #print(t1)
                t2 = t1[1]
                data_x.append(t2)
                t = float(t2)
                if t > max_x_err:
                    max_x_err = t;
            if ("yError" in line):
                t1 = line.split('yError')
                data_y.append(t1[1])
                t2 = t1[1]
                data_y.append(t2)
                t = float(t2)
                if t > max_y_err:
                    max_y_err = t;
            if ("headingError" in line):
                t1 = line.split('headingError')
                t2 = t1[1]
                data_h.append(t2)
                t = float(t2)
                if t > max_heading_err:
                    max_heading_err = t;
        #####################################
        if ("DriveVelocityPIDTuner: error 0" in line):
            #print(t);
            t1 = line.split('error 0');
            data_v_err.append(t1[1])
        if ("DriveVelocityPIDTuner: targetVelocity" in line):
            t1 = line.split('targetVelocity')
            data_v_target.append(t1[1])
        if ("DriveVelocityPIDTuner: velocity 0" in line):
            t1 = line.split('velocity 0');
            t2 = t1[1];
            data_v_actual.append(t2)
            t = float(t2.strip());
            t = abs(t)
            if t > max_v:
                max_v = t;
        #############################################
        if ("setMotorPowers" in line):
            t = line.split('setMotorPowers');
            t1 = t[1].strip().split(' ');
            #print(t1)
            t2 = t1[1]
            data_power.append(t2)
            t = float(t2)
            if t>max_power:
                max_power=t;
        ###########################################
        if ("RobotCore" in line) and ("START - OPMODE" in line):
            t = line.split('OPMODE');
            t1 = t[1].strip().split(' ')
            p_name=t1[0]

    for i in range(len(data_x)):
        print(data_x[i].strip(), " ", data_y[i].strip(), " ", data_h[i].strip());

    for i in range(len(data_v_err)):
        print(data_v_err[i].strip(), " ", data_v_target[i].strip(), " ", data_v_actual[i].strip());

if print_summary != 0:
    print("===============summary==========================")
    print("programe ran: ", p_name)
    print("max power to wheel: ", max_power)
    print("max_x_err: ", max_x_err)
    print("max_y_err: ", max_y_err)
    print("max_heading_err ", max_heading_err, math.degrees(max_heading_err))
    print("max_velocity: ", max_v)
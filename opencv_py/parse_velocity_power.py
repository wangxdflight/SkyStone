import sys

import string
num_wheels=4;

filepath = sys.argv[1]
with open(filepath) as fp:
    line = fp.readline()
    cnt = 1
    data=[];
    v_count=0;
    while line:
        #print("Line {}: {}".format(cnt, line.strip()))
        line = fp.readline()
        if "SampleMecanumDriveREV: SampleMecanumDriveREV created" in line:
            t=line.split(' ');
            #print(t)
            start_time=t[1];
            #print(t[1])
            print("start time "+start_time)
        elif "RobotCore: thread: ...terminating 'LinearOpMode main'" in line:
            t=line.split(' ');
            #print(t)
            end_time=t[1];
            print("end time "+end_time)
        elif "DriveVelocityPIDTuner96.0: getWheelVelocities" in line:
            count=0;
            while count<num_wheels:
                line1=fp.readline();
                #print(line1)
                t1=line1.split(' ')
                data.append(t1[8].strip('\n'));
                count=count+1;

        elif "DriveVelocityPIDTuner96.0: getMotorPowers" in line:
            count=0;
            while count<num_wheels:
                line1=fp.readline();
                #print(line1)
                t1=line1.split(' ')
                data.append(t1[8].strip('\n'));
                count=count+1;

        elif "DriveVelocityPIDTuner96.0: targetVelocity" in line:
                #dprint(line)
                t1=line.split(' ')
                data.append(t1[7].strip('\n'));
                v_count=0;

        elif "DriveVelocityPIDTuner96.0: velocity" in line:
                #print(line)
                t1=line.split(' ')
                data.append(t1[8].strip('\n'));

        elif "DriveVelocityPIDTuner96.0: error" in line:
            #print(line1)
            t1=line.split(' ')
            data.append(t1[8].strip('\n'));
            v_count=v_count+1;

            if v_count==num_wheels:
                for i in range(len(data)):
                    print(data[i], end=' ')
                print ("")
                data=[];

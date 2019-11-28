import sys

import string
num_wheels=4;

filepath = sys.argv[1]
sessions=0;
start_time=""
end_time=""
with open(filepath) as fp:
    line = fp.readline()
    cnt = 1
    data=[];
    v_count=0;
    started=0;
    while line:
        #print("Line {}: {}".format(cnt, line.strip()))
        line = fp.readline()
        if "DriveVelocityPIDTuner: targetVelocity" in line:
            #print(line)
            t1=line.split('DriveVelocityPIDTuner')
            t2=t1[1].split(' ')
            #print(t2)
            #print(t2[2])
            data.append(t2[2].strip('\n'));
            v_count=0;
        elif "DriveVelocityPIDTuner: getMotorPowers" in line:
            v_count=0;
            #print(line)
            while v_count<num_wheels:
                line1=fp.readline();
                #print(line1)
                if "SampleMecanumDriveBase: " not in line1:
                    break;
                t=line1.split("SampleMecanumDriveBase:");
                t=t[1].split(" ");
                #print(t)
                #data.append(t[3].strip('\n'));
                v_count=v_count+1;
        elif "DriveVelocityPIDTuner: velocity" in line:
            t1=line.split('DriveVelocityPIDTuner: velocity')
            #print(t1)
            t2=t1[1].split(' ')
            #print(t2)
            data.append(t2[2].strip('\n'));
            v_count=v_count+1;

            if v_count==num_wheels:
                for i in range(len(data)):
                    print(data[i], end=' ');
                print();
                v_count=0;
                data=[];


#print("start time "+start_time + " " + start_time)
#print("end time "+end_time + " " + end_time)
#print(sessions);

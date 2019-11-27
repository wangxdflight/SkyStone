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
        if ("Robocol : received command" in line) and ("DriveVelocityPIDTuner" in line):
            t=line.split(' ');
            #print(t)
            start_time=t[1];
            #print(t[1])
            started=1;
            sessions=sessions+1;
        elif ("RobotCore: thread: ...terminating 'LinearOpMode main'" in line) and (started==1):
            t=line.split(' ');
            #print(t)
            end_time=t[1];

            started=0;
        elif started<=1:
            if "DriveVelocityPIDTuner: getWheelVelocities" in line:
                count=0;
                while count<num_wheels:
                    line1=fp.readline();
                    #print(line1)
                    t1=line1.split('SampleMecanumDriveBase')
                    t2=t1[1].split(' ')
                    #print(t2)
                    data.append(t2[3].strip('\n'));
                    count=count+1;

            elif "DriveVelocityPIDTuner: getMotorPowers" in line:
                count=0;
                while count<num_wheels:
                    line1=fp.readline();
                    #print(line1)
                    t1=line1.split('SampleMecanumDriveBase')
                    t2=t1[1].split(' ')
                    #print(t2)
                    data.append(t2[3].strip('\n'));
                    count=count+1;

            elif "DriveVelocityPIDTuner: targetVelocity" in line:
                #print(line)
                t1=line.split('DriveVelocityPIDTuner')
                t2=t1[1].split(' ')
                #print(t2)
                data.append(t2[2].strip('\n'));
                v_count=0;

            elif "DriveVelocityPIDTuner: velocity" in line:
                #print(line)
                t1=line.split('DriveVelocityPIDTuner')
                t2=t1[1].split(' ')
                #print(t2)
                data.append(t2[3].strip('\n'));

            elif "DriveVelocityPIDTuner: error" in line:
                #print(line)
                t1=line.split('DriveVelocityPIDTuner')
                t2=t1[1].split(' ')
                #print(t2)
                data.append(t2[3].strip('\n'));
                v_count=v_count+1;

                if v_count==num_wheels:
                    for i in range(len(data)):
                        print(data[i], end=' ')
                    print ("")
                    data=[];

print("start time "+start_time + " " + start_time)
print("end time "+end_time + " " + end_time)
print(sessions);

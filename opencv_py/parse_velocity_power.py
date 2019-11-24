import string
num_wheels=4;

filepath = "../velocity_power_test.log"
with open(filepath) as fp:
    line = fp.readline()
    cnt = 1
    data=[];
    while line:
        #print("Line {}: {}".format(cnt, line.strip()))
        line = fp.readline()

        if "DriveVelocityPIDTuner: getWheelVelocities" in line:
            count=0;
            while count<num_wheels:
                line1=fp.readline();
                #print(line1)
                t1=line1.split(' ')
                data.append(t1[10].strip('\n'));
                count=count+1;

        if "DriveVelocityPIDTuner: getMotorPowers" in line:
            count=0;
            while count<num_wheels:
                line1=fp.readline();
                #print(line1)
                t1=line1.split(' ')
                data.append(t1[10].strip('\n'));
                count=count+1;

            for i in range(len(data)):
                print(data[i], end=' ')
            print ("")
            data=[];

import sys
import string

num_wheels=4;
filepath = sys.argv[1]
with open(filepath) as fp:
    line = fp.readline()
    data=[];
    count=0;
    while line:
        if "ManualParamTest: velocities" in line:
            v_count=0;
            while v_count<num_wheels:
                t1=fp.readline();
                t2=t1.split("SampleMecanumDriveBase:")
                t3=t2[1].split(' ')
                #print(t3)
                data.append(t3[3].strip('\n'));
                v_count=v_count+1;
        elif "StandardTrackingWheelLocalizer: encoderTicksToInches:  ticks:" in line:
            t1=line.split("StandardTrackingWheelLocalizer: encoderTicksToInches:  ticks:")
            t2=t1[1].split(" ")
            data.append(t2[1])
            data.append(t2[3].strip('\n'));
        # SampleMecanumDriveREV: getWheelPositions: position: 1478.0 inches: 77.46859548706092
        elif "SampleMecanumDriveREV: getWheelPositions" in line:
            #print(line)
            #
            t1=line.split('SampleMecanumDriveREV: getWheelPositions: position:')
            t2=t1[1].split(' ')
            #print(t2)
            data.append(t2[1]);
            data.append(t2[3].strip('\n'));
            count=count+1;

            if count==num_wheels:
                for i in range(len(data)):
                    print(data[i], end=' ');

                print();
                data=[];
                count=0;
                start=0;


        line=fp.readline();



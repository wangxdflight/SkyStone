import sys
import string

num_wheels=4;
filepath = sys.argv[1]
with open(filepath) as fp:
    line = fp.readline()
    data=[];
    count=0;
    while line:
        if "TestMaxVelocity: velocities" in line:
            v_count=0;
            while v_count<4:
                t1=fp.readline();
                t2=t1.split("SampleMecanumDriveBase:")
                t3=t2[1].split(' ')
                #print(t3)
                data.append(t3[3].strip('\n'));
                v_count=v_count+1;

            if v_count==4:
                for i in range(len(data)):
                    print(data[i], end=' ');

                print();
                data=[];
                count=0;
                start=0;


        line=fp.readline();



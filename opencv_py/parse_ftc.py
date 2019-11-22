import string


filepath = "../2.log"
with open(filepath) as fp:
    line = fp.readline()
    cnt = 1
    while line:
        #print("Line {}: {}".format(cnt, line.strip()))
        line = fp.readline()

        if "setMotorPowers" in line:
            powers=[];
            count=0;
            while count<4:
                line1=fp.readline();
                t1=line1.split(' ')
                powers.append(t1[9].strip('\n'));
                count=count+1;
            print(powers)

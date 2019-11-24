

import string

data=[];
data_y=[];
data_h=[]; #heading;
filepath = "../25.log"
with open(filepath) as fp:
    line = fp.readline()
    while line:
        line = fp.readline()
        if "SampleMecanumDriveBase: update: x" in line:
            data=[];
            t=line.split(' ')
            t=t[8].strip('\n');
            data.append(t)
        elif "SampleMecanumDriveBase: xError" in line:
            t=line.split(' ')
            t=t[7].strip('\n');

            data.append(t)


        elif "SampleMecanumDriveBase: y " in line:
            data_y=[];
            t=line.split(' ');
            #print(t)
            t=t[7].strip('\n');

            data_y.append(t)
        elif "SampleMecanumDriveBase: yError " in line:
            t=line.split(' ')
            t=t[7].strip('\n')
            data_y.append(t)


        elif "SampleMecanumDriveBase: heading " in line:
            data_h=[];
            t=line.split(' ');
            #print(t)
            t=t[7].strip('\n');

            data_h.append(t)
        elif "SampleMecanumDriveBase: headingError " in line:
            t=line.split(' ')
            #print(t)
            t=t[7].strip('\n')
            data_h.append(t)

            for i in range(len(data)):
                print(data[i], end=' ');
            for i in range(len(data_y)):
                print(data_y[i], end=' ');
            for i in range(len(data_h)):
                print(data_h[i], end=' ');
            print();

            data=[];
            data_y=[];
            data_h=[];

import sys
import string

data=[];
data_y=[];
data_h=[]; #heading;
filepath = sys.argv[1];
with open(filepath) as fp:
	line = fp.readline()
	while line:
		line = fp.readline()
		if "SampleMecanumDriveBase: update: x" in line:
			data=[];
			#print(line);
			t=line.split('SampleMecanumDriveBase: update: x')
			t=t[1].split(' ')
			#print(t);
			t=t[1].strip('\n');
			data.append(t)
		elif "SampleMecanumDriveBase: xError" in line:
			t=line.split('SampleMecanumDriveBase: xError');
			t=t[1].split(' ')
			t=t[1].strip('\n');
			data.append(t)

		elif "SampleMecanumDriveBase: y " in line:
			data_y=[];
			t=line.split('SampleMecanumDriveBase: y ');
			#print(t)
			t=t[1].split(' ')
			t=t[0].strip('\n');

			data_y.append(t)
		elif "SampleMecanumDriveBase: yError " in line:
			t=line.split('SampleMecanumDriveBase: yError ');
			t=t[1].split(' ')
			t=t[0].strip('\n');
			data_y.append(t)


		elif "SampleMecanumDriveBase: heading " in line:
			data_h=[];
			t=line.split('SampleMecanumDriveBase: heading ');
			t=t[1].split(' ')
			t=t[0].strip('\n');

			data_h.append(t)
		elif "SampleMecanumDriveBase: headingError " in line:
			t=line.split('SampleMecanumDriveBase: headingError ');
			t=t[1].split(' ')
			t=t[0].strip('\n');
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
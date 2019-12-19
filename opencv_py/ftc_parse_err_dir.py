import os
from subprocess import call
for subdir, dirs, files in os.walk("."):
    for file in files:
        #print os.path.join(subdir, file)
        filepath = subdir + os.sep + file

        if filepath.endswith(".log"):
            print(filepath)
            cmd_line = "cat " + filepath + " |grep 'Robocol : received command: CMD_INIT_OP_MODE'";
            os.system(cmd_line);

            cmd_line = "cat " + filepath + " |grep debug.ftc";
            os.system(cmd_line);

            cmd_line = "cat " + filepath + " |grep xErr | tail";
            os.system(cmd_line);

            cmd_line = "cat " + filepath + " |grep yErr | tail"
            os.system(cmd_line);

            cmd_line = "cat " + filepath + " |grep headingErr | tail"
            os.system(cmd_line);
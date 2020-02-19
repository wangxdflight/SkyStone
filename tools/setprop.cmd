adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_red1.xml /sdcard/FIRST
adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_red2.xml /sdcard/FIRST
adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_red3.xml /sdcard/FIRST
adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_blue1.xml /sdcard/FIRST
adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_blue2.xml /sdcard/FIRST
adb push TeamCode\src\main\java\org\firstinspires\ftc\teamcode\All\path_blue3.xml /sdcard/FIRST

adb shell setprop debug.ftc.logging 1
adb shell setprop debug.ftc.enable_arm 1
adb shell setprop debug.ftc.skystonePos 2
adb shell setprop debug.ftc.pause 1
adb shell setprop debug.ftc.vuforia 0
adb shell setprop debug.ftc.bulk 0
adb shell setprop debug.ftc.imu 1
adb shell setprop debug.ftc.resetfollow 1
adb shell setprop debug.ftc.forceOdom 1
adb shell setprop debug.ftc.diagsplit 1
adb shell setprop debug.ftc.imuInterval 10
rem ---------------- odometry parameters -----------------
adb shell setprop debug.ftc.odom 0
adb shell setprop debug.ftc.odoTicksPerRevLeft 1565
adb shell setprop debug.ftc.odoTicksPerRevRight 1565
adb shell setprop debug.ftc.odoTicksPerRevFront 1565
adb shell setprop debug.ftc.odomTrackwidth 14.8
adb shell setprop debug.ftc.odomForwardOffset -5.5
rem ------------------
adb shell setprop debug.ftc.brake 1
adb shell setprop debug.ftc.recreateDrv 0
adb shell setprop debug.ftc.drvCorrect 0
rem (-24, 24) for strafe diagonal
adb shell setprop debug.ftc.distance -40
adb shell setprop debug.ftc.distance0 50
adb shell setprop debug.ftc.strafeDiag 1
adb shell setprop debug.ftc.trackwidth 14.2
adb shell setprop debug.ftc.maxVel 70.0
adb shell setprop debug.ftc.maxAccel 35.0
adb shell setprop debug.ftc.strafeMaxVel 30.0
adb shell setprop debug.ftc.strafeMaxAccel 10.0
rem ------------------ velocity PID ------------------
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.0111
adb shell setprop debug.ftc.kP 1.72
adb shell setprop debug.ftc.kI 0.172
adb shell setprop debug.ftc.kD 0.0
rem ----------------------- transitional PID ---------------------
adb shell setprop debug.ftc.txP 5.0
adb shell setprop debug.ftc.txI 0.5
adb shell setprop debug.ftc.txD 0.00001
adb shell setprop debug.ftc.tyP 5.0
adb shell setprop debug.ftc.tyI 10.0
adb shell setprop debug.ftc.tyD 0.00001
rem ------------------------ heading PID ------------------
adb shell setprop debug.ftc.hP 10.0
adb shell setprop debug.ftc.hI 0.5
adb shell setprop debug.ftc.hD 0.00001
rem --------------- strafing paramters, PIDs ----------------------------
adb shell setprop debug.ftc.stxP 20
adb shell setprop debug.ftc.stxI 1
adb shell setprop debug.ftc.stxD 0.75
adb shell setprop debug.ftc.styP 15
adb shell setprop debug.ftc.styI 0.5
adb shell setprop debug.ftc.styD 1
adb shell setprop debug.ftc.shP 6
adb shell setprop debug.ftc.shI 2
adb shell setprop debug.ftc.shD 0.4
rem ------------------- raw paramters of strafing, not using roadrunner --------------
rem time in seconds needed per inch;
adb shell setprop debug.ftc.strafeTimeDistanceRat 0.093
adb shell setprop debug.ftc.strafeMotorPower 0.19
adb shell setprop debug.ftc.rear_ratio 1.105
adb shell getprop |grep debug.ftc
rem adb logcat -s VrApi
IF "%1"=="1" (
exit
)
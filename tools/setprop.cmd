adb shell setprop debug.ftc.imu 1
adb shell setprop debug.ftc.odom 1
adb shell setprop debug.ftc.vuforia 0
adb shell setprop debug.ftc.brake 0
adb shell setprop debug.ftc.bulk 0
adb shell setprop debug.ftc.recreateDrv 0
adb shell setprop debug.ftc.drvCorrect 0
adb shell setprop debug.ftc.distance 60
adb shell setprop debug.ftc.distance0 24
adb shell setprop debug.ftc.strafeDiag 1
adb shell setprop debug.ftc.trackwidth 14.2
adb shell setprop debug.ftc.maxVel 90.0
adb shell setprop debug.ftc.maxAccel 35.0
adb shell setprop debug.ftc.strafeMaxVel 30.0
adb shell setprop debug.ftc.strafeMaxAccel 15.0
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.0111
rem 0.0111 for 4 wheel
adb shell setprop debug.ftc.kP 0.1
adb shell setprop debug.ftc.kI 1.52
adb shell setprop debug.ftc.kD 3.0
rem drivetrain paramters;
adb shell setprop debug.ftc.txP 6
adb shell setprop debug.ftc.txI 0.6
adb shell setprop debug.ftc.txD 0.75
adb shell setprop debug.ftc.tyP 5.0
adb shell setprop debug.ftc.tyI 0.001
adb shell setprop debug.ftc.tyD 0.00001
adb shell setprop debug.ftc.hP 6
adb shell setprop debug.ftc.hI 2
adb shell setprop debug.ftc.hD 0.4
rem strafing paramters ---------------------
adb shell setprop debug.ftc.stxP 20
adb shell setprop debug.ftc.stxI 1
adb shell setprop debug.ftc.stxD 0.75
adb shell setprop debug.ftc.styP 15
adb shell setprop debug.ftc.styI 0.5
adb shell setprop debug.ftc.styD 1
adb shell setprop debug.ftc.shP 6
adb shell setprop debug.ftc.shI 2
adb shell setprop debug.ftc.shD 0.4
rem time in seconds needed per inch;
adb shell setprop debug.ftc.strafeTimeDistanceRat 0.093
adb shell setprop debug.ftc.strafeMotorPower 0.19
adb shell setprop debug.ftc.rear_ratio 1.105
adb shell setprop debug.ftc.odoTicksPerRev 1565
adb shell setprop debug.ftc.odomTrackwidth 14.8
adb shell setprop debug.ftc.odomForwardOffset -5.5
adb shell getprop |grep debug.ftc
rem adb logcat -s VrApi
IF "%1"=="1" (
exit
)

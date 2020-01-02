adb shell setprop debug.ftc.distance 72
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.0090
rem 0.0111 for 4 wheel
adb shell setprop debug.ftc.kP 0.1
adb shell setprop debug.ftc.kI 1.52
adb shell setprop debug.ftc.kD 3.0
adb shell setprop debug.ftc.imu 0
adb shell setprop debug.ftc.odom 1
adb shell setprop debug.ftc.brake 0
adb shell setprop debug.ftc.trackwidth 14.2
adb shell setprop debug.ftc.txP 2
adb shell setprop debug.ftc.txI 0.3
adb shell setprop debug.ftc.txD 0.12
adb shell setprop debug.ftc.tyP 0.3
adb shell setprop debug.ftc.tyI 0.5
adb shell setprop debug.ftc.tyD 1.1
adb shell setprop debug.ftc.hP 2
adb shell setprop debug.ftc.hI 1.5
adb shell setprop debug.ftc.hD 0.2
adb shell setprop debug.ftc.odoTicksPerRev 1550
adb shell setprop debug.ftc.odomTrackwidth 14.8
adb shell setprop debug.ftc.odomForwardOffset -5.5
adb shell getprop |grep debug.ftc
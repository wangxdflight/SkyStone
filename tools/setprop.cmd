adb shell setprop debug.ftc.distance 45
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.00825
adb shell setprop debug.ftc.kP 35
adb shell setprop debug.ftc.kI 0.5
adb shell setprop debug.ftc.kD 2.5
adb shell setprop debug.ftc.imu 0
adb shell setprop debug.ftc.odom 0
adb shell setprop debug.ftc.trackwidth 17
adb shell setprop debug.ftc.tP 0
adb shell setprop debug.ftc.tI 0
adb shell setprop debug.ftc.tD 0
adb shell setprop debug.ftc.hP 0
adb shell setprop debug.ftc.hI 0
adb shell setprop debug.ftc.hD 0
adb shell setprop debug.ftc.odomTrackwidth 15.5
adb shell setprop debug.ftc.odomForwardOffset -5.49
adb shell getprop |grep debug.ftc
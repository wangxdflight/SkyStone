adb shell setprop debug.ftc.distance 40
rem smaller does make straight test go shorter distance;
adb shell setprop debug.ftc.kV 0.00825
adb shell setprop debug.ftc.kP 35
adb shell setprop debug.ftc.kI 0.5
adb shell setprop debug.ftc.kD 2.5
adb shell setprop debug.ftc.imu 1
adb shell setprop debug.ftc.odom 1
adb shell setprop debug.ftc.trackwidth 17
adb shell setprop debug.ftc.tP 5
adb shell setprop debug.ftc.tI 0
adb shell setprop debug.ftc.tD 0
adb shell setprop debug.ftc.hP 10
adb shell setprop debug.ftc.hI -2
adb shell setprop debug.ftc.hD 25
adb shell setprop debug.ftc.odomTrackwidth 14.8
adb shell setprop debug.ftc.odomForwardOffset 5.5
adb shell getprop |grep debug.ftc
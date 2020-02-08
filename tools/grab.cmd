adb wait-for-device
adb shell getprop debug.ftc.grab
adb shell setprop debug.ftc.grab 1
TIMEOUT /T 10
adb shell setprop debug.ftc.grab 0
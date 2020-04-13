adb root
adb wait-for-device
adb push TeamCode\src\main\jniLibs\armeabi-v7a\libOpenCvNative.so /sdcard/FIRST
adb shell chmod 777 /sdcard/FIRST/libOpenCvNative.so
dir TeamCode\build\outputs\apk\debug\TeamCode-debug.apk
adb shell pm list packages -f -3|grep ftcrobotcontroller
adb install -t TeamCode\build\outputs\apk\debug\TeamCode-debug.apk
adb shell pm list packages -f -3|grep ftcrobotcontroller
rem setprop.cmd
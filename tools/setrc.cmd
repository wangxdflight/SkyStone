adb root
adb wait-for-device
adb push TeamCode\src\main\jniLibs\armeabi-v7a\libOpenCvNative.so /sdcard/FIRST
adb shell chmod 777 /sdcard/FIRST/libOpenCvNative.so
adb install -t TeamCode\build\outputs\apk\debug\TeamCode-debug.apk
setprop.cmd
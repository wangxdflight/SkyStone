adb root
adb tcpip 5555
adb wait-for-device
adb shell ifconfig wlan0
adb connect 192.168.1.233:5555
adb devices

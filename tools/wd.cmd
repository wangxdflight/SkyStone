@echo off
@echo "adb over wifi direct"
adb root
adb wait-for-device

@echo off
adb wait-for-device
adb tcpip 5555
adb wait-for-device
set var=192.168.49.1
ping %var%
start "wifi keepalive" ping %var% -t
adb connect %var%:5555

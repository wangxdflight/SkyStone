@echo cmd %filename
adb shell screencap -p /sdcard/screen.png
if "%1"=="" (
	adb pull /sdcard/screen.png 
) else (
	adb pull /sdcard/screen.png %1.png
)
adb shell rm /sdcard/screen.png
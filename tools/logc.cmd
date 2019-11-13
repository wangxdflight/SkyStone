@echo off
@echo cmd logfilename ps_name
IF "%2"==""  (
	adb shell ps |grep com.qualcomm.ftcrobotcontroller>ps.txt
) ELSE (
	adb shell ps |grep %2 >ps.txt
)
set /p ftcrobot=<ps.txt
IF "%ftcrobot%"=="" (
	@echo "com.qualcomm.ftcrobotcontroller has died!!!"
	goto XX
)
echo %ftcrobot%

for /F "tokens=1,2 delims=: " %%a in ("%ftcrobot%") do (
   @rem echo %%a
   echo %%b
   set var=%%b
)
IF "%1"==""  (
	adb logcat -b all -v color --pid %var% 
) ELSE (
	adb logcat -b all  --pid %var% |wtee %1.log
)

:XX
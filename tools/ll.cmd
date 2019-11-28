@echo off
if "%1"=="" (
dir /od 
) ELSE (
dir /od *.%1
)
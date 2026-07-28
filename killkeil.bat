@echo off
setlocal

set "FOUND=0"
for %%P in (
    UV4.exe
    UV4Launcher.exe
    UV3.exe
    UV2.exe
    armcc.exe
    armclang.exe
    armlink.exe
    armasm.exe
    fromelf.exe
) do (
    tasklist /FI "IMAGENAME eq %%P" 2>nul | find /I "%%P" >nul
    if not errorlevel 1 (
        set "FOUND=1"
        taskkill /F /T /IM "%%P" >nul 2>&1
        if errorlevel 1 (
            echo Failed to stop %%P
        ) else (
            echo Stopped %%P
        )
    )
)

if "%FOUND%"=="0" echo No Keil processes are running.

endlocal
exit /b 0

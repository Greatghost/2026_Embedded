@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "FOUND=0"
set "FAILED=0"

echo Stopping Keil processes...
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
            echo [ERROR] Failed to stop %%P
            set "FAILED=1"
        ) else (
            echo [OK] Stopped %%P
        )
    )
)

if "!FOUND!"=="0" echo [OK] No Keil processes are running.

if "!FAILED!"=="1" (
    echo [ERROR] Build cache was not cleared because a Keil process could not be stopped.
    exit /b 1
)

echo Clearing Keil build cache...
call :CleanDir "%~dp0Sentry_Steer\Chassis\project\Objects"
call :CleanDir "%~dp0Sentry_Steer\Chassis\project\Listings"
call :CleanDir "%~dp0Sentry_Steer\Gimbal\MDK-ARM\sentry_chassis_test"

if "!FAILED!"=="1" (
    echo [ERROR] Some build cache files could not be removed.
    exit /b 1
)

echo [OK] Keil build cache cleared.
exit /b 0

:CleanDir
if not exist "%~1\" (
    echo [SKIP] Cache directory not found: %~1
    exit /b 0
)

del /F /S /Q "%~1\*" >nul 2>&1
for /D %%D in ("%~1\*") do rd /S /Q "%%~fD" >nul 2>&1

dir /A /B "%~1" 2>nul | findstr "^" >nul
if not errorlevel 1 (
    echo [ERROR] Cache directory is not empty: %~1
    set "FAILED=1"
) else (
    echo [OK] Cleared: %~1
)
exit /b 0

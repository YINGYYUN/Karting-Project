@echo off
rem Debug build. Output: bin\Debug\net9.0-windows\KartHostLite.exe (+ .dll)
rem dotnet lookup order: env DOTNET -> workspace private SDK -> PATH.
rem NOTE: keep this file ASCII-only. Chinese text here breaks cmd parsing under GBK codepage.
setlocal
if not "%DOTNET%"=="" goto :found
set "DOTNET=D:\Project\Code\CYT4BB7\Karting\.tools\dotnet\dotnet.exe"
if exist "%DOTNET%" goto :found
set "DOTNET=dotnet"

:found
set DOTNET_CLI_TELEMETRY_OPTOUT=1
set DOTNET_NOLOGO=1
"%DOTNET%" build -c Debug -v m
if errorlevel 1 goto :fail
echo.
echo Build OK: bin\Debug\net9.0-windows\KartHostLite.exe
echo Run it with run.bat ^(do NOT double-click the exe in bin, it needs .NET 9^).
exit /b 0

:fail
echo.
echo Build FAILED.
exit /b 1

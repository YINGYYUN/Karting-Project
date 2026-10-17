@echo off
rem Publish a self-contained single-file build: dist\KartHostLite.exe (~113 MB, embeds .NET 9).
rem The private SDK is only needed for this publish step; the produced exe runs anywhere on Win10/11 x64.
rem NOTE: keep this file ASCII-only. Chinese text here breaks cmd parsing under GBK codepage.
setlocal
if not "%DOTNET%"=="" goto :found
set "DOTNET=D:\Project\Code\CYT4BB7\Karting\.tools\dotnet\dotnet.exe"
if exist "%DOTNET%" goto :found
set "DOTNET=dotnet"

:found
set DOTNET_CLI_TELEMETRY_OPTOUT=1
set DOTNET_NOLOGO=1
"%DOTNET%" publish -c Release -r win-x64 --self-contained true -p:PublishSingleFile=true -p:IncludeNativeLibrariesForSelfExtract=true -o "%~dp0dist" -v m
if errorlevel 1 goto :fail
echo.
echo Published: %~dp0dist\KartHostLite.exe
exit /b 0

:fail
echo.
echo Publish FAILED.
exit /b 1

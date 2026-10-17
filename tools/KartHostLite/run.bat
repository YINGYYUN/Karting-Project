@echo off
rem Launch the host app. Extra args are passed through (e.g. run.bat --selftest).
rem Priority: dist\KartHostLite.exe (self-contained) -> dotnet + dll (private SDK, .NET 9).
rem NOTE: keep this file ASCII-only. Chinese text here breaks cmd parsing under GBK codepage.
setlocal
set "HERE=%~dp0"

if exist "%HERE%dist\KartHostLite.exe" goto :selfcontained

if not "%DOTNET%"=="" goto :found
set "DOTNET=D:\Project\Code\CYT4BB7\Karting\.tools\dotnet\dotnet.exe"
if exist "%DOTNET%" goto :found
set "DOTNET=dotnet"

:found
set DOTNET_CLI_TELEMETRY_OPTOUT=1
set DOTNET_NOLOGO=1
set "DLL=%HERE%bin\Debug\net9.0-windows\KartHostLite.dll"
if not exist "%DLL%" goto :nobuild
"%DOTNET%" "%DLL%" %*
exit /b %errorlevel%

:selfcontained
"%HERE%dist\KartHostLite.exe" %*
exit /b %errorlevel%

:nobuild
echo Not built yet. Run build.bat ^(debug^) or publish.bat ^(single-file dist^).
exit /b 1

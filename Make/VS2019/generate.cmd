@echo off
set SHELLOPTS=igncr
pushd "%~dp0"
cmake -G "Visual Studio 17 2022" -A x64 -S ..\Common -B ..\..\Build\Windows\CMake
bash.exe ./generate
if exist B-Human.lnk goto linkExists
powershell "$s=(New-Object -COM WScript.Shell).CreateShortcut('%CD%\B-Human.lnk');$s.TargetPath='%CD%\..\..\Build\Windows\CMake\B-Human.sln';$s.Save()"
:linkExists
popd
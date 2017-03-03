@echo off
set "sourceDir=%~dp0"
set "currentDir=%cd%"
set "scriptDir=%sourceDir%scripts\python"
:build
echo Build start.
cd "%sourceDir%"
conan install -u -s arch="x86" -s build_type="Release" -s compiler="Visual Studio" -s compiler.runtime="MD" -s compiler.version="12" -o build_develop=True
conan build
cd "%currentDir%"
echo Build done.
pause
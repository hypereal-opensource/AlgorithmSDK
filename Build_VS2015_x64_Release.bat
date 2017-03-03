@echo off
set "sourceDir=%~dp0"
set "currentDir=%cd%"
set "scriptDir=%sourceDir%scripts\python"
:build
echo Build start.
cd "%sourceDir%"
conan install -u -s arch="x86_64" -s build_type="Release" -s compiler="Visual Studio" -s compiler.runtime="MD" -s compiler.version="14" -o build_develop=True
conan build
cd "%currentDir%"
echo Build done.
pause
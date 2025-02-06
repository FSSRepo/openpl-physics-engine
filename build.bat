@echo off
if not exist build (
    mkdir build
)
if not exist "build\Release\openpl.dll" (
    cmake -S . -B build -DPL_JAVA=ON
    cmake --build build --config Release
)

if "%1"=="--x86" (
    if not exist build_x86 (
        mkdir build_x86
    )
    if not exist "build_x86\Release\openpl.dll" (
        cmake -S . -B build_x86 -DPL_JAVA=ON -A Win32
        cmake --build build_x86 --config Release
    )
)

if not exist dist (
    mkdir dist
)
if not exist build\classes (
    mkdir build\classes
)

dir /b /s java-bindings\*.java > sources.txt
javac -d build\classes @sources.txt
del sources.txt
jar cvf "dist/openpl.jar" -C "build\classes" .

copy build\Release\openpl.dll dist\openpl64.dll

if "%1"=="--x86" (
    copy build_x86\Release\openpl.dll dist\openpl.dll
)

echo Done!!
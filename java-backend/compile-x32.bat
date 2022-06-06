@echo off
@echo Compilando...
g++ -c wrapper.cpp -o wrapper.o -I"C:\mingw64\include" -I"C:\mingw64\x86_64-w64-mingw32\include" -I"..\src" -I"..\src\bullet" -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" -m32 -DBUILDING_DLL=1
g++ -shared wrapper.o -o bin\openpl.dll -L"C:\mingw64\x86_64-w64-mingw32\lib32" -static-libgcc ..\windows\bin\Release\libopenpl.a -m32 -Wl,--output-def,bin\libopenpljni.def,--out-implib,bin\libopenpljni.a,--add-stdcall-alias
@echo Se completo la compilacion. Escribe 'e' para terminar
@set /p Op=
if %Op% == "e" goto exit
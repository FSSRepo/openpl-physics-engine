@echo off
@echo Compilando...
if not exist "bin" mkdir "bin"
g++ -c wrapper.cpp -o wrapper.o -I"..\src" -I"..\src\bullet" -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" -m32 -DBUILDING_DLL=1
g++ -shared wrapper.o -o bin\openpl.dll -static-libgcc ..\windows\bin\Release\libopenpl.a -m32 -Wl,--output-def,bin\libopenpljni.def,--out-implib,bin\libopenpljni.a,--add-stdcall-alias
@echo Se completo la compilacion. Escribe 'e' para terminar
@set /p Op=
if %Op% == "e" goto exit
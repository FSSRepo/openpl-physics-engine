@echo off
@echo Compilando...
if not exist "bin" mkdir "bin"
g++ -c wrapper.cpp -o wrapper.o -I"C:\mingw64\x86_64-w64-mingw32\include" -I"..\src" -I"..\src\bullet" -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" -m64 -DBUILDING_DLL=1
g++ -shared wrapper.o -o bin\openpl64.dll -static-libgcc ..\windows\bin\Release64\libopenpl64.a -m64 -Wl,--output-def,bin\libopenpl64jni.def,--out-implib,bin\libopenpl64jni.a,--add-stdcall-alias
@echo Se completo la compilacion. Escribe 'e' para terminar
@set /p Op=
if %Op% == "e" goto exit
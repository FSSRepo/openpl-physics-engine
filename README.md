# OpenPL
 Physics Engine API in C++ similar to OpenGL

# Docs

[Open Documentation](https://fssrepo.github.io/openpl-physics-engine/)

# Build

First, clone this library:

```bash
git clone https://github.com/FSSRepo/openpl-physics-engine.git
cd openpl-physics-engine
```

You will need CMake to build binaries:

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

## Java bindings

To use java bindings build like this, you need JDK installed and JAVA_HOME environment variable defined:

```bash
mkdir build
cd build
cmake .. -DPL_JAVA=ON
cmake --build . --config Release
```

## Android

To compile the library for Android, run the following commands:

```bash
cd android
gradlew assembleRelease
```

Your AAR Library is `android/app/build/outputs/aar/`
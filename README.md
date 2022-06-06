# OpenPL
 Physics Engine API in C++ similar to OpenGL
 
# Docs

[Open Documentation](https://fssrepo.github.io/openpl-physics-engine/)

# Requeriments

MinGW latest version (/bin added PATH Environment variable)

Code Blocks 17.12 to up

# Compiling

Windows

Open Code Blocks projects in windows folder and build it

<type><arch>

type
dynamic = Build a Dynamic Link Library (.dll)
static = Build a Static Library (.a) (just works in MinGW)

32 = x86 (32 bits) architecture
64 = x64 (64 bits) architecture

dynamic32 (.dll for 32 bits)
dynamic64 (.dll for 64 bits)

static32 (.a for 32 bits)
static64 (.a for 64 bits)

Java

This requires before has been built static32 or static64 projects (depends of the architecture that you want)

```bash
cd java-backend
```

Build 32 bits library:
```bash
compile-x32
```
Build 64 bits library:
```bash
compile-x64
```

Android

Just open the android project and build

Requires NDK and Cmake installed

# Usage


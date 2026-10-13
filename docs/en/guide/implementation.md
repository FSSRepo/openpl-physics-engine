---
layout: default
title: Implementation
parent: Guide
nav_order: 2
---

# Implementation

OpenPL can be compiled on any platform. Just include it in your C++ or Java project and
start building amazing things.

## Code::Blocks

It is the IDE where the API is easiest to integrate, since the binaries are compiled to
be used directly in this IDE.

### Binaries

| Platform | Downloads |
| :------- | :-------- |
| Windows | 32-bit · 64-bit |
| Linux | 32-bit · 64-bit |

### Implementation styles

- **As a static library** — included inside the executable; no complementary DLL file is
  needed.
- **As a dynamic library** — a DLL file is included alongside the executable. There is no
  big difference; it is a personal decision.
- **As source code** — the API is included and can be modified to make it more
  customizable and convenient to use.

## Android Studio

Implementation for developing on the Android platform.

### Binaries

Android arm-v7, arm64, x86, x86_64.

- **As native code** — consists of including all the code in a single C++ library, which
  can be your own project's library if you are natively developing an application that
  needs it.
- **As a Java backend** — consists of including the precompiled library files and a
  precompiled Java library, so you only add it to the project dependencies.

## NetBeans

Implementation in the Java IDE for desktop projects.

### Binaries

openpl-java 32-bit · 64-bit.

The implementation consists of creating a library to access the native methods of the API.

## Visual Studio

{: .warning }
Visual Studio integration documentation is **coming soon**.

{: .note }
Binary download links are published on
[GitHub Releases](https://github.com/FSSRepo/openpl-physics-engine/releases).

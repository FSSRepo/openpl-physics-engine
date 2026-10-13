---
layout: default
title: Specifications
parent: Guide
nav_order: 1
---

# Specifications

## Supported languages

The API runs natively thanks to C++, so the executed code is hardware accelerated. It
also ships with a Java backend, so it can be used in many kinds of projects:

- **C++**
- **Java**

## Platforms and execution mode

It is supported on every operating system, as long as the project code is compiled for
the matching architecture and executable:

| Platform | Native (C++) | Backend (Java) | Architecture |
| :------- | :----------: | :------------: | :----------- |
| Android | Yes | Yes | arm-v7, arm64, x86, x86_64 |
| Windows | Yes | Yes | x86, amd64 |
| Linux | Yes | Yes | x86, amd64 |

## It is very stable

It has been tested across many examples and projects. It turns out to be an innovative
and efficient solution; on top of that, for people starting out in game programming it
makes it very easy to implement physics without having to start from scratch.

{: .note }
OpenPL is built on **Bullet Physics 3.25**, a mature physics engine widely used in the
industry.

---
layout: default
title: Implementación
parent: Guía
nav_order: 2
---

# Implementación

OpenPL puede compilarse en cualquier plataforma. Solo inclúyelo en tu proyecto de C++ o
Java y empieza a crear cosas increíbles.

## Code::Blocks

Es el IDE donde la API se integra con mayor facilidad, ya que los binarios están
compilados para usarse directamente en este IDE.

### Binarios

| Plataforma | Descargas |
| :--------- | :-------- |
| Windows | 32 bits · 64 bits |
| Linux | 32 bits · 64 bits |

### Formas de implementación

- **Como librería estática** — se incluye dentro del ejecutable; no hace falta un archivo
  DLL complementario.
- **Como librería dinámica** — se incluye un archivo DLL como complemento del ejecutable.
  No hay gran diferencia; es una decisión personal.
- **Como código fuente** — se incluye y se puede modificar la API para hacerla más
  personalizable y cómoda de usar.

## Android Studio

Implementación para desarrollar en la plataforma Android.

### Binarios

Android arm-v7, arm64, x86, x86_64.

- **Como código nativo** — consiste en incluir todo el código en una sola librería C++,
  que puede ser la de tu propio proyecto si desarrollas de forma nativa una aplicación que
  lo requiera.
- **Como backend Java** — consiste en incluir los archivos de librería precompilados y una
  librería Java precompilada, para solo añadirla a las dependencias del proyecto.

## NetBeans

Implementación en el IDE de Java para proyectos de escritorio.

### Binarios

openpl-java 32 bits · 64 bits.

La implementación consiste en crear una librería para acceder a los métodos nativos de la
API.

## Visual Studio

{: .warning }
Documentación de integración en Visual Studio **próximamente**.

{: .note }
Los enlaces de descarga de binarios se publican en
[GitHub Releases](https://github.com/FSSRepo/openpl-physics-engine/releases).

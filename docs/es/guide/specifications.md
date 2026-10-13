---
layout: default
title: Especificaciones
parent: Guía
nav_order: 1
---

# Especificaciones

## Lenguajes soportados

La API funciona de forma nativa gracias a C++, por tanto el código ejecutado está
acelerado por hardware. También cuenta con un backend en Java, por lo que puede usarse
en múltiples proyectos:

- **C++**
- **Java**

## Plataformas y modo de ejecución

Es soportado por todos los sistemas operativos, siempre y cuando el código del proyecto
esté compilado en la arquitectura y ejecutable correspondiente:

| Plataforma | Nativo (C++) | Backend (Java) | Arquitectura |
| :--------- | :----------: | :------------: | :----------- |
| Android | Sí | Sí | arm-v7, arm64, x86, x86_64 |
| Windows | Sí | Sí | x86, amd64 |
| Linux | Sí | Sí | x86, amd64 |

## Es muy estable

Ha sido probado en múltiples ejemplos y proyectos. Resulta ser una solución innovadora y
eficiente; además, para quienes empiezan en el mundo de la programación de videojuegos
será muy fácil implementar las físicas sin necesidad de comenzar desde cero.

{: .note }
OpenPL está construido sobre **Bullet Physics 3.25**, un motor de físicas maduro y
ampliamente utilizado en la industria.

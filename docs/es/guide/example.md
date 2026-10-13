---
layout: default
title: Ejemplo
parent: Guía
nav_order: 4
---

# Ejemplo

Un esqueleto mínimo de OpenPL. Observa el patrón **en cola** de la API: primero se
**genera** un recurso, luego se le pasan sus parámetros y finalmente se **crea**
indicando el tipo concreto.

```cpp
#include <openpl/bullet.h>
#include <iostream>

using namespace std;

/*
    Fast Smart System
    cross-platform open source project
*/

int main() {
    // 1. Crear el contexto global.
    if (!plCreateContext()) {
        cout << "Can't create the context" << endl;
        return 1;
    }

    // 2. Generar un recurso en cola (una forma).
    int shape = plGenShape();

    // 3. Pasar sus parámetros.
    plShapef(PL_RADIUS, 3.0f);

    // 4. Crear el recurso indicando el tipo concreto.
    plCreate(PL_SPHERE_SHAPE);

    // 5. Seleccionar el recurso generado.
    plBindShape(shape);

    // 6. Liberar el contexto.
    plDestroyContext();
    return 0;
}
```

{: .note }
`plGenShape()` devuelve un **id** que se referencia después con `plBindShape()`.
Consultar la [referencia de la API](../api/index.html) para ver el resto de funciones.

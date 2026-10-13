---
title: Formas de colisión
parent: Referencia API
grand_parent: Inicio
nav_order: 5
---

# Formas de colisión

<div class="openpl-api" markdown="1">

## `plGenShape`

```c
Pluint plGenShape();
```

## `plBindShape`

Enlazar para usar una forma de colision.

```c
void plBindShape(Pluint shape);
```

### Parámetros

| Nombre | |
| --- | --- |
| `shape` | La id de la forma de colision a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar una forma de colisionasegurate de que no haya sido enlazado previamente, si es asi para desenlazar shape debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si ya hay una forma de colision enlazada previamente. |

## `plShapei`

Poner un valor a un parametro especifico

```c
void plShapei(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_USER_POINTER, PL_ADD_CHILD_SHAPE, PL_REMOVE_CHILD_SHAPE, PL_OPTIMIZE_CONVEX_HULL. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_USER_POINTER poner un puntero de usuario. value debe ser el user pointer que desees.
- PL_ADD_CHILD_SHAPE agregar una forma de colision a un compuesto. value debe ser la forma de colision a agregar.
- PL_REMOVE_CHILD_SHAPE remover una forma de colision a un compuesto. value debe ser la forma de colision a remover.
- PL_OPTIMIZE_CONVEX_HULL optimizar forma de convex hull. value debe ser 1 o 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si param no es permitido. |

## `plShapef`

Poner un valor a un parametro especifico

```c
void plShapef(Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_HEIGHT, PL_RADIUS, PL_PLANE_CONSTANT. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_HEIGHT poner altura de un cono, capsula o cilindro . value nunca debe ser 0.
- PL_RADIUS poner radio de una esfera o de los extremos de un cilindro, capsula, cono. value nunca debe ser 0.
- PL_PLANE_CONSTANT poner altura de un plano estatico. value debe ser el valor equivalente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si se intenta asignar uno de estos valores despues de crear la forma de colision. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plShape3f`

Poner un valor a un parametro especifico

```c
void plShape3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_EXTENT, PL_PLANE_NORMAL, PL_LOCAL_SCALING. |
| `x y z` | Los valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_EXTENT poner la extension. x y z debe ser el vector(x,y,z).
- PL_PLANE_NORMAL asignar la direccion de colision de un plano estatico. x y z debe ser el vector(x,y,z) del normal.
- PL_LOCAL_SCALING escalar una forma de colision. x y z debe ser el vector(x,y,z) de escala.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plShapefv`

Poner un valor a un parametro especifico

```c
void plShapefv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_TRANSFORM. |
| `values` | Los valores asignados. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_TRANSFORM poner la transformacion para la forma de colision.values debe ser un matrix4x4 de transformacion.lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plBufferData`

Establecer un buffer de datos.

```c
void plBufferData(Plenum type, Plsizei size, void* data, Plint copy);
```

### Parámetros

| Nombre | |
| --- | --- |
| `type` | Especifica el tipo de dato a introducir. Puede ser PL_VERTEX_BUFFER, PL_INDEX_BUFFER. |
| `size` | La cantidad de vertices si el tipo es PL_VERTEX_BUFFER, la cantidad de indices si es PL_INDEX_BUFFER. |
| `data` | Puntero de entrada de datos binarios. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por type:
- PL_VERTEX_BUFFER buffer de vertices en el orden xyz, con puntos flotantes de 32 bits. data debe ser un arreglo de puntos flotantes (float*)(float[]).
- PL_INDEX_BUFFER buffer de indices en el orden abc, con enteros de 16 bits. data debe ser un arreglo de enteros de 16 bits (short[]) (short*).
- Los datos que se introdujeron en ningun momento del ciclo de vida deben ser modificados.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | data nunca debe ser nulo. |
| `—` | size nunca debe ser 0. |
| `—` | Los errores consecuentes de esto se presentan al usar otras funciones relacionadas. |

## `plGetShapei`

Obtener un valor a un parametro especifico

```c
void plGetShapei(Pluint shape, Plenum param, Plint* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SHAPE_TYPE, PL_USER_POINTER, PL_NUM_CHILD_SHAPES. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SHAPE_TYPE obtener el tipo de forma de colision.
- PL_USER_POINTER obtener el user pointer.
- PL_NUM_CHILD_SHAPES obtener el numero de children. Debe ser compound shape.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si la forma de colision no existe o el id es incorrecto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetShapefv`

Obtener un valor a un parametro especifico

```c
void plGetShapefv(Pluint shape, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `shape` | El id de la forma de colision |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_TRANSFORM, |
| `values` | Valores para obtener. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_TRANSFORM poner la transformacion.values debe ser un matrix4x4 de transformacion.lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

</div>

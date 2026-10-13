---
title: Cuerpos blandos
parent: Referencia API
grand_parent: Inicio
nav_order: 9
---

# Cuerpos blandos

<div class="openpl-api" markdown="1">

## `plGenSoftBody`

Generar un cuerpo blando y encolarlo.

```c
Pluint plGenSoftBody(Plenum type);
```

### Parámetros

| Nombre | |
| --- | --- |
| `type` | Especifica el tipo de cuerpo blando a crear. Puede ser PL_SOFTBODY_ROPE, PL_SOFTBODY_CLOTH, PL_SOFTBODY_ELLIPSOID, PL_SOFTBODY_MESH. |

### Notas

{: .note }
- Esta funcion retorna una id y crea una cola de variables necesarias antes de crear una instancia del objeto.
- El tipo PL_SOFTBODY_MESH requiere definir antes los buffers con plBufferData.
- Para materializar el cuerpo blando debes llamar a plCreate(PL_SOFT_BODY).

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | Esta funcion retorna 0 si no hay contexto o hay un error global. |

## `plBindSoftBody`

Enlazar para usar un cuerpo blando.

```c
void plBindSoftBody(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | La id del cuerpo blando a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar un cuerpo blando asegurate de que no haya sido enlazado previamente, si es asi para desenlazar indx debe ser 0.

## `plDeleteSoftBody`

Eliminar un cuerpo blando.

```c
void plDeleteSoftBody(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | La id del cuerpo blando a eliminar. |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando ya fue eliminado o no existe. |

## `plSoftBodyi`

Poner un valor a un parametro especifico

```c
void plSoftBodyi(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SB_RESOLUTION, PL_SB_FIXED_NODES, PL_SB_ITERATIONS, PL_SB_ANCHOR_BODY, PL_SB_AERO_MODEL, PL_SB_RANDOMIZE_CONSTRAINTS. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_RESOLUTION establece la resolucion de nodos en X al generar el cuerpo. Solo valido en la cola.
- PL_SB_FIXED_NODES numero de nodos fijos. Solo valido en la cola.
- PL_SB_ITERATIONS iteraciones del solver (piterations y viterations).
- PL_SB_ANCHOR_BODY enlaza el cuerpo blando a un cuerpo rigido; value debe ser el id del cuerpo rigido.
- PL_SB_AERO_MODEL modelo aerodinamico (valor del enum btSoftBody::eAeroModel).
- PL_SB_RANDOMIZE_CONSTRAINTS value debe ser 1 (PL_TRUE) para aleatorizar las restricciones o 0 si no. Solo valido en la cola.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando no existe o el id es incorrecto. |

## `plSoftBodyf`

Poner un valor a un parametro especifico

```c
void plSoftBodyf(Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SB_RADIUS, PL_SB_TOTAL_MASS, PL_SB_PRESSURE, PL_SB_STIFFNESS, PL_SB_DAMPING, PL_SB_VELOCITY_CORRECTION. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_RADIUS radio de la esfera de colision de cada nodo. Solo valido en la cola.
- PL_SB_TOTAL_MASS masa total del cuerpo blando.
- PL_SB_PRESSURE presion interna (kPR).
- PL_SB_STIFFNESS rigidez del material (kLST, kAST y kVST).
- PL_SB_DAMPING amortiguacion (kDP).
- PL_SB_VELOCITY_CORRECTION factor de correccion de velocidad (kVCF).

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando no existe o el id es incorrecto. |

## `plSoftBody3f`

Poner un valor a un parametro especifico

```c
void plSoftBody3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SB_POSITION_A, PL_SB_POSITION_B, PL_SB_POSITION_C, PL_SB_POSITION_D. |
| `x y z` | Los valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_POSITION_A primer punto (x,y,z) que define la posicion del cuerpo blando.
- PL_SB_POSITION_B segundo punto (x,y,z).
- PL_SB_POSITION_C tercer punto (x,y,z).
- PL_SB_POSITION_D cuarto punto (x,y,z).
- Estos parametros solo son validos durante la cola (antes de plCreate).

## `plSoftBodyfv`

Poner un valor a un parametro especifico

```c
void plSoftBodyfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SB_RESOLUTION, PL_SB_ANCHOR_NODES. |
| `values` | El arreglo de valores. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_RESOLUTION establece la resolucion X e Y; values[0] es X y values[1] es Y, lenght debe ser 2 o mas.
- PL_SB_ANCHOR_NODES indices de los nodos anclados; values contiene los indices y lenght la cantidad.

## `plGetSoftBodyi`

Obtener un valor entero a traves de un parametro especifico.

```c
void plGetSoftBodyi(Pluint indx, Plenum param, Plint* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | ID del cuerpo blando. |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_SB_NUM_NODES. |
| `value` | El valor obtenido. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_NUM_NODES obtiene el numero de nodos del cuerpo blando.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando no existe o el id es incorrecto. |

## `plGetSoftBodyf`

Obtener un valor flotante a traves de un parametro especifico.

```c
void plGetSoftBodyf(Pluint indx, Plenum param, Plfloat* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | ID del cuerpo blando. |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_SB_VOLUME, PL_SB_TOTAL_MASS. |
| `value` | El valor obtenido. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_VOLUME obtiene el volumen del cuerpo blando.
- PL_SB_TOTAL_MASS obtiene la masa total del cuerpo blando.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando no existe o el id es incorrecto. |

## `plGetSoftBodyfv`

Obtener un arreglo de valores a traves de un parametro especifico.

```c
void plGetSoftBodyfv(Pluint indx, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | ID del cuerpo blando. |
| `param` | Especifica el parametro para obtener los valores. Puede ser PL_SB_NODE_POSITION. |
| `values` | Valores para obtener. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SB_NODE_POSITION obtiene las posiciones (x,y,z) de todos los nodos; values debe tener espacio para num_nodes * 3 y lenght debe ser num_nodes * 3 o mas.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo blando no existe o el id es incorrecto. |
| `PL_INVALID_VALUE` | si el arreglo de salida es demasiado pequeño. |

</div>

---
title: Personajes cinemáticos
parent: Referencia API
grand_parent: Inicio
nav_order: 8
---

# Personajes cinemáticos

<div class="openpl-api" markdown="1">

## `plGenCharacter`

Generar un personaje o character.

```c
Pluint plGenCharacter();
```

### Notas

{: .note }
- Esta funcion retorna una id y crea una cola de variables necesarias antes de crear una instancia del objeto.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si hay una cola pendiente para crear un personaje o character. |

## `plBindCharacter`

Enlazar para usar un personaje.

```c
void plBindCharacter(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `chr` | La id del personaje a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar un personaje asegurate de que no haya sido enlazado previamente, si es asi para desenlazar chr debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si ya hay un personaje enlazado previamente. |

## `plDeleteCharacter`

Eliminar un character o personaje.

```c
void plDeleteCharacter(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `chr` | La id del character a eliminar. |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el character esta enlazado o esta siendo usado. Tambien puede ser causado por que el character que tratas de eliminar ya no existe. |

## `plCharacterf`

Poner un valor a un parametro especifico

```c
void plCharacterf(Plenum param, Plfloat val);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CHAR_STEP_HEIGHT, PL_CHAR_SPHERE, PL_CHAR_ANGULAR_DAMPING, PL_CHAR_LINEAR_DAMPING, PL_CHAR_PRE_STEP, PL_CHAR_PLAYER_STEP, PL_CHAR_JUMP_SPEED, PL_CHAR_FALL_SPEED, PL_CHAR_JUMP, PL_CHAR_MAX_SLOPE, PL_CHAR_MAX_JUMP_HEIGHT, PL_CHAR_MAX_PENETRATION_DEPTH. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CHAR_STEP_HEIGHT poner altura de paso.
- PL_CHAR_SPHERE poner una esfera.
- PL_CHAR_ANGULAR_DAMPING poner el angular damping
- PL_CHAR_ANGULAR_DAMPING poner el linear damping
- PL_CHAR_PRE_STEP pre step*
- PL_CHAR_PLAYER_STEP player step*
- PL_CHAR_JUMP_SPEED poner velocidad de salto
- PL_CHAR_FALL_SPEED poner velocidad de caida
- PL_CHAR_JUMP saltar
- PL_CHAR_MAX_SLOPE max slope*
- PL_CHAR_MAX_JUMP_HEIGHT altura maxima de salto
- PL_CHAR_MAX_PENETRATION_DEPTH profundidad de penetracion.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el personaje no existe o el id es incorrecto. |

## `plCharacter3f`

Poner un valor a un parametro especifico

```c
void plCharacter3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CHAR_UPVEC, PL_CHAR_CAPSULE_X, PL_CHAR_CAPSULE_Y, PL_CHAR_CAPSULE_Z, PL_CHAR_BOX, PL_CHAR_WALK_DIRECTION, PL_CHAR_JUMP PL_CHAR_APPLY_IMPULSE, PL_CHAR_ANGULAR_VELOCITY, PL_CHAR_LINEAR_VELOCITY. |
| `x y z` | Los valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CHAR_UPVEC poner el vector de altura.
- PL_CHAR_CAPSULE_X poner una capsula orientada en el eje x.
- PL_CHAR_CAPSULE_Y poner una capsula orientada en el eje y.
- PL_CHAR_CAPSULE_Z poner una capsula orientada en el eje z.
- PL_CHAR_BOX poner una caja.
- PL_CHAR_WALK_DIRECTION poner la direccion de camino.
- PL_CHAR_JUMP saltar a una direccion.
- PL_CHAR_APPLY_IMPULSE aplicar un impulso.
- PL_CHAR_ANGULAR_VELOCITY poner la velocidad angular.
- PL_CHAR_LINEAR_VELOCITY poner la velocidad lineal.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el personaje no existe o el id es incorrecto. |

## `plCharacterfv`

```c
void plCharacterfv(Plenum param, Plfloat* values);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Plenum |
| `values` | Plfloat* |

## `plGetCharacterf`

Obtener un valor a un parametro especifico

```c
void plGetCharacterf(Pluint indx, Plenum param, Plfloat* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | ID del personaje |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CHAR_CAN_JUMP, |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CHAR_CAN_JUMP obtener 1 o 0 si el personaje puede saltar.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No es muy comun que se oplengan errores |

## `plGetCharacterb`

```c
PLbool plGetCharacterb(Pluint indx, Plenum param);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | Pluint |
| `param` | Plenum |

## `plGetCharacterfv`

Obtener un valor a un parametro especifico

```c
void plGetCharacterfv(Pluint indx, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `indx` | ID del personaje |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_TRANSFORM, |
| `values` | Valores para obtener. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_TRANSFORM obtener la transformacion del personaje. values es una Matrix4x4. lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |

</div>

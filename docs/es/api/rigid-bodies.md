---
title: Cuerpos rígidos
parent: Referencia API
grand_parent: Inicio
nav_order: 4
---

# Cuerpos rígidos

<div class="openpl-api" markdown="1">

## `plRigidBodyi`

Poner un valor a un parametro especifico

```c
void plRigidBodyi(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_USER_POINTER, PL_COLLISION_CALLBACK_FILTER, PL_COLLISION_CALLBACK_FLAG, PL_USER_INDEX, PL_ACTIVATION_STATE, PL_ADD_COLLISION_FLAG, PL_COLLISION_SHAPE, PL_ANISOTROPIC_ROLLING_FRICTION. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_USER_POINTER poner un puntero de usuario. value debe ser el user pointer que desees.
- PL_COLLISION_CALLBACK_FILTER poner el filtro para la prueba de contacto. value debe ser el filtrado.
- PL_COLLISION_CALLBACK_FLAG asignar caracteristica para filtrar en la prueba de contacto. value debe ser la caracteristica
- PL_USER_INDEX poner un indice de usuario. value debe ser el indice.
- PL_ACTIVATION_STATE cambiar el estado de activacion. value debe ser PL_DIS_DACTIVATION si quieres que el cuerpo rigido permanesca activo todo el tiempo, PL_DIS_SIMULATION si quieres que el cuerpo rigido se ignore durante la simulacion, PL_ACTIVATE si quieres activar el cuerpo rigido.
- PL_ADD_COLLISION_FLAG asignar carasteristicas de colision. value si quieres que el cuerpo rigido, PL_CF_KINEMATIC_OBJECT sea cinematico, PL_CF_STATIC_OBJECT estatico, PL_CF_CUSTOM_MATERIAL_CALLBACK materiales personalizados.
- PL_COLLISION_SHAPE usar Dbvt broadphase. value debe ser 0.
- PL_ANISOTROPIC_ROLLING_FRICTION poner el rozamiento anisotropico. value debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | si intentas usar PL_COLLISION_CALLBACK_FILTER, PL_COLLISION_CALLBACK_FLAG sin antes activar la prueba de contactos o sin poner el user pointer. |

## `plRigidBodyf`

Poner un valor a un parametro especifico

```c
void plRigidBodyf(Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_MASS, PL_ANGULAR_FACTOR, PL_ROLLING_FRICTION, PL_SPINNING_FRICTION, PL_FRICTION, PL_RESTITUTION. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_MASS poner masa en kilogramos. value debe ser la masa en kilogramos.
- PL_ANGULAR_FACTOR factor angular. value debe ser el factor angular.
- PL_ROLLING_FRICTION friccion de rodadura. value debe ser el valor equivalente
- PL_SPINNING_FRICTION friccion giratoria. value debe ser el valor equivalente.
- PL_FRICTION friccion. value debe ser el valor equivalente.
- PL_RESTITUTION coeficiente de restitucion. value debe ser el valor equivalente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plRigidBody3f`

Poner un valor a un parametro especifico

```c
void plRigidBody3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_POSITION, PL_MOTION_STATE_POSITION, PL_DAMPING, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_APPLY_CENTRAL_FORCE PL_APPLY_TORQUE, PL_APPLY_CENTRAL_IMPULSE, PL_APPLY_TORQUE_IMPULSE, PL_ANISOTROPIC_FRICTION, PL_COL_INTERP_LINEAR_VEL, PL_COL_INTERP_ANGULAR_VEL. |
| `x y z` | Los valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_POSITION poner la posicion del centro de masa. x y z debe ser el vector(x,y,z) de posicion.
- PL_MOTION_STATE_POSITION poner la posicion del estado de movimiento. x y z debe ser el vector(x,y,z) de posicion.
- PL_DAMPING poner factor de amortiguacion lineal y giratoria. x debe ser el factor amortiguacion lineal. y debe ser el factor amortiguacion giratoria.
- PL_LINEAR_FACTOR poner el factor lineal. x y z es el vector(x,y,z).
- PL_ANGULAR_FACTOR poner el factor de giracion. x y z es el vector(x,y,z).
- PL_LOCAL_INERTIA poner el eje de inercia local. x y z debe ser el vector(x,y,z) de inercia.
- PL_APPLY_CENTRAL_FORCE aplicar una fuerza central. x y z es el vector de fuerza a aplicar(x,y,z).
- PL_APPLY_TORQUE aplicar torque. x y z es el vector de torque(x,y,z).
- PL_APPLY_CENTRAL_IMPULSE aplicar impulso central. x y z debe ser el vector(x,y,z) de impulso.
- PL_APPLY_TORQUE_IMPULSE aplicar impulso con torque. x y z es el vector(x,y,z) de impulso.
- PL_ANISOTROPIC_FRICTION poner el factor de friccion anisotropica. x y z es el vector(x,y,z).
- PL_COL_INTERP_LINEAR_VEL poner la velocidad lineal de interpolacion.x y z es el vector(x,y,z).
- PL_COL_INTERP_ANGULAR_VEL poner la velocidad giratoria de interpolacion. x y z es el vector(x,y,z).

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plRigidBodyfv`

Poner un valor a un parametro especifico

```c
void plRigidBodyfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_POSITION, PL_MOTION_STATE_POSITION, PL_BASIS, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_APPLY_CENTRAL_FORCE PL_APPLY_TORQUE, PL_APPLY_CENTRAL_IMPULSE, PL_APPLY_TORQUE_IMPULSE, PL_ORIENTATION, PL_TRANSFORM, PL_MOTION_STATE_TRANSFORM, PL_COL_WORLD_TRANSFORM, PL_COL_INTERP_WORLD_TRANSFORM. |
| `values` | Los valores asignados. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_POSITION poner la posicion del centro de masa. values debe ser el vector de posicion. lenght debe ser 3
- PL_MOTION_STATE_POSITION poner la posicion del estado de movimiento. values debe ser el vector(x,y,z) de posicion. lenght debe ser 3
- PL_BASIS poner un nuevo basis. values debe ser una matrix3x3. lenght debe ser 3
- PL_LINEAR_FACTOR poner el factor lineal. values es el vector.lenght debe ser 3
- PL_ANGULAR_FACTOR poner el factor giracion. values es el vector.lenght debe ser 3
- PL_LOCAL_INERTIA poner el eje de inercia local. values debe ser el vector de inercia.lenght debe ser 3
- PL_APPLY_CENTRAL_FORCE aplicar una fuerza central. values es el vector de fuerza a aplicar. lenght debe ser 3
- PL_APPLY_TORQUE aplicar torque. values es el vector de torque. lenght debe ser 3
- PL_APPLY_CENTRAL_IMPULSE aplicar impulso central. values debe ser el vector de impulso.lenght debe ser 3
- PL_APPLY_TORQUE_IMPULSE aplicar impulso con torque. values es el vector de impulso.lenght debe ser 3
- PL_ORIENTATION poner la orientacion o rotacion. values es el quaternion de rotacion.lenght debe ser 4
- PL_TRANSFORM poner la transformacion.values debe ser un matrix4x4 de transformacion.lenght debe ser 16
- PL_MOTION_STATE_TRANSFORM poner la transformacion en el estado de movimiento. values debe ser un matrix4x4 de transformacion.lenght debe ser 16
- PL_COL_WORLD_TRANSFORM poner la transformacion del mundo.values debe ser un matrix4x4 de transformacion.lenght debe ser 16
- PL_COL_INTERP_WORLD_TRANSFORM poner la transformacion del mundo interpolada. values debe ser un matrix4x4 de transformacion.lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetRigidBodyi`

Obtener un valor por un parametro especifico

```c
void plGetRigidBodyi(Pluint body, Plenum param, Plint* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_USER_POINTER, PL_USER_INDEX, PL_IS_IN_WORLD. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_USER_POINTER obtener el puntero de usuario.
- PL_USER_INDEX obtener el indice de usuario.
- PL_IS_IN_WORLD obtener si el cuerpo rigido esta en el mundo dinamico o no.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetRigidBodyf`

Obtener un valor a un parametro especifico

```c
void plGetRigidBodyf(Pluint body, Plenum param, Plfloat* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_MASS, PL_ANGULAR_FACTOR, PL_ROLLING_FRICTION, PL_SPINNING_FRICTION, PL_FRICTION, PL_RESTITUTION. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_MASS poner masa en kilogramos. value debe ser la masa en kilogramos.
- PL_ANGULAR_FACTOR factor angular. value debe ser el factor angular.
- PL_ROLLING_FRICTION friccion de rodadura. value debe ser el valor equivalente
- PL_SPINNING_FRICTION friccion giratoria. value debe ser el valor equivalente.
- PL_FRICTION friccion. value debe ser el valor equivalente.
- PL_RESTITUTION coeficiente de restitucion. value debe ser el valor equivalente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetRigidBody3f`

Obtener un valor a un parametro especifico

```c
void plGetRigidBody3f(Pluint body, Plenum param, Plfloat* v1, Plfloat* v2, Plfloat* v3);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_POSITION, PL_DAMPING, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_TOTAL_FORCE PL_TOTAL_TORQUE, PL_LINEAR_VELOCITY, PL_ANGULAR_VELOCITY. |
| `v1 v2 v3` | Los valores oplenidos. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_POSITION obtener la posicion del centro de masa. v1 v2 v3 es el vector(x,y,z) de posicion.
- PL_DAMPING obtener el factor de amortiguacion lineal y giratorio. v1 es el valor de amortiguacion lineal y v2 es el valor de amortiguacion giratorio.
- PL_LINEAR_FACTOR obtener el factor lineal. v1 v2 v3 es el vector(x,y,z).
- PL_ANGULAR_FACTOR obtener el factor giracion. v1 v2 v3 es el vector(x,y,z).
- PL_LOCAL_INERTIA obtener el eje de inercia local. v1 v2 v3 es el vector(x,y,z) .
- PL_TOTAL_FORCE obtener el total de las fuerzas. v1 v2 v3 es el vector(x,y,z).
- PL_TOTAL_TORQUE obtener el total del torque. v1 v2 v3 es el vector(x,y,z).
- PL_LINEAR_VELOCITY obtener la velocidad lineal. v1 v2 v3 es el vector(x,y,z).
- PL_ANGULAR_VELOCITY obtener la velocidad angular. v1 v2 v3 es el vector(x,y,z).

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetRigidBodyfv`

Obtener un valor a un parametro especifico

```c
void plGetRigidBodyfv(Pluint body, Plenum param, Plfloat* values, Plint lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `body` | El id del cuerpo rigido |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_POSITION, PL_MOTION_STATE_POSITION, PL_BASIS, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_TOTAL_TORQUE PL_TOTAL_FORCE, PL_LINEAR_VELOCITY, PL_ANGULAR_VELOCITY, PL_ORIENTATION, PL_TRANSFORM, PL_MOTION_STATE_TRANSFORM. |
| `values` | Valores para obtener. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_POSITION obtener la posicion del centro de masa. values es un vector3. lenght debe ser 3
- PL_MOTION_STATE_POSITION obtener la posicion del estado de movimiento. values es un vector3. lenght debe ser 3
- PL_BASIS obtener el basis de la transformacion del centro de masa. values debe ser una matrix3x3. lenght debe ser 3
- PL_LINEAR_FACTOR obtener el factor lineal. values es un vector3.lenght debe ser 3
- PL_ANGULAR_FACTOR obtener el factor giracion. values es un vector3.lenght debe ser 3
- PL_LOCAL_INERTIA obtener el eje de inercia local. values es un vector3.lenght debe ser 3
- PL_TOTAL_FORCE obtener el total de la fuerza. values es un vector3.lenght debe ser 3
- PL_TOTAL_TORQUE obtener el total del torque. values es un vector3. lenght debe ser 3
- PL_LINEAR_VELOCITY obtener la velocidad lineal. values es un vector3. lenght debe ser 3
- PL_ANGULAR_VELOCITY obtener la velocidad angular o giratoria. values es un vector3.lenght debe ser 3
- PL_ORIENTATION poner la orientacion o rotacion. values es el quaternion de rotacion.lenght debe ser 4
- PL_TRANSFORM poner la transformacion.values debe ser un matrix4x4 de transformacion.lenght debe ser 16
- PL_MOTION_STATE_TRANSFORM poner la transformacion en el estado de movimiento. values debe ser un matrix4x4 de transformacion.lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

</div>

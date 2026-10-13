---
title: Mundo dinámico
parent: Referencia API
grand_parent: Inicio
nav_order: 2
---

# Mundo dinámico

<div class="openpl-api" markdown="1">

## `plDynamicWorldi`

Poner un valor a un parametro especifico

```c
void plDynamicWorldi(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_ADD_RIGID_BODY, PL_REMOVE_RIGID_BODY, PL_ADD_CONSTRAINT, PL_REMOVE_CONSTRAINT, PL_INIT_VEHICLE_RAYCASTER, PL_INIT_CHARACTER, PL_ADD_CHARACTER, PL_CLEAR_RAY_RESULTS, PL_USE_DBVT, PL_CONTACT_TEST, PL_ADD_FLAG_RAY_TEST, PL_USER_POINTER, PL_DEBUG_MODE. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_ADD_RIGID_BODY agregar un cuerpo rigido. value debe ser el id del cuerpo rigido ya creado.
- PL_REMOVE_RIGID_BODY remover un cuerpo rigido. value debe ser el id del cuerpo rigido a remover.
- PL_ADD_CONSTRAINT agregar una restriccion. value debe ser el id de la restriccion ya creada.
- PL_REMOVE_CONSTRAINT remover una restriccion. value debe ser el id de la restriccion a remover.
- PL_INIT_VEHICLE_RAYCASTER inicializar el vehivle ray caster. value debe ser 0.
- PL_CLEAR_RAY_RESULTS eliminar los resultados de la prueba de rayo. value debe ser 0.
- PL_USE_DBVT usar Dbvt broadphase. value debe ser 0.
- PL_CONTACT_TEST value debe ser 1 si quieres usar la prueba de contactos o 0 si no quieres.
- PL_ADD_FLAG_RAY_TEST agregar filtro a la prueba de ray casting value puede ser PL_RF_FLT_BACK_FACES, PL_RF_KP_UNFLIPPED_NORM, PL_RF_SS_CCR, PL_RF_GJK_CCR.
- PL_USER_POINTER buscar y enlazar el cuerpo rigido apartir del user pointer.value debe ser el user pointer.
- PL_DEBUG_MODE value debe ser 1 si quieres usar el modo de depuracion o 0 si no quieres.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido, restriccion, personaje no existen o el id es incorrecto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plDynamicWorld3f`

Poner un valor a un parametro especifico

```c
void plDynamicWorld3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_GRAVITY, PL_USE_AXIS_SWEEP3. |
| `v0 v1 v2` | Valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_GRAVITY poner el vector de gravedad, por defecto es (0,-9.8,0). v0 v1 v2 deben ser el vector de gravedad (x,y,x).
- PL_USE_AXIS_SWEEP3 usar AxisSweep3 v0 v1 v2 deben ser el vector de extension.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si el parametro especificado no es permitido. |

## `plDynamicWorldfv`

Poner un valor a un parametro especifico

```c
void plDynamicWorldfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_GRAVITY, PL_RAY_CLOSEST_TEST, PL_RAY_ALL_TEST, PL_USE_AXIS_SWEEP3. |
| `values` | El arreglo de valores |
| `lenght` | Longitud del arreglo. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_GRAVITY poner vector de gravedad. values debe ser el vector (x,y,z).
- PL_RAY_CLOSEST_TEST hacer test de ray casting, el objeto mas cercano. values debe ser el vector ray origin (x,y,z) y ray end (x,y,z).
- PL_RAY_ALL_TEST hacer test de ray casting, todos los objetos sin excepciones.values debe ser el vector ray origin (x,y,z) y ray end (x,y,z).
- PL_USE_AXIS_SWEEP3 usar AxisSweep3 broadphase values debe ser el vector min y max.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_VALUE` | si el valor lenght es 0 o no es correcto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetDynamicWorldi`

Obtener un valor a traves de un parametro especifico.

```c
void plGetDynamicWorldi(Plenum param, Plint* value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_NUM_RIGID_BODY, PL_RAY_BODY_RESULT, PL_NUM_RAY_RESULT. |
| `value` | El valor para obtener. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_NUM_RIGID_BODY obtener el numero de cuerpos rigidos.
- PL_RAY_BODY_RESULT obtener el cuerpo rigido que fue golpeado por el raycasting.
- PL_NUM_RAY_RESULT obtener el numero de objetos golpeados por el raycasting.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetDynamicWorld3f`

Obtener un valor a traves de un parametro especifico.

```c
void plGetDynamicWorld3f(Plenum param, Plfloat* v1, Plfloat* v2, Plfloat* v3);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para obtener los valores. Puede ser PL_GRAVITY, PL_RAY_POINT_RESULT. |
| `v0 v1 v2` | Los valores para obtener. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_GRAVITY obtener el vector de gravedad en el orden (x,y,z).
- PL_RAY_POINT_RESULT obtener el punto de golpe del ray casting (x,y,z).

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetDynamicWorldiv`

Obtener un valor a traves de un parametro especifico.

```c
void plGetDynamicWorldiv(Plenum param, Plint* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para obtener los valores. Puede ser PL_RAY_BODY_RESULT. |
| `values` | El arreglo de valores |
| `lenght` | Longitud del arreglo. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_RAY_BODY_RESULT obtener las id de los cuerpos rigidos que fueron golpeados por el ray casting.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si se intenta obtener el parametro PL_RAY_BODY_RESULT sin antes haber hecho una prueba de raycasting. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |
| `PL_INVALID_VALUE` | si lenght es 0 o incorrecto. |

## `plGetDynamicWorldfv`

Obtener un valor a traves de un parametro especifico.

```c
void plGetDynamicWorldfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_RAY_POINT_RESULT, PL_RAY_NORMAL_RESULT. |
| `values` | El arreglo de valores |
| `lenght` | Longitud del arreglo. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_RAY_POINT_RESULT obtener los puntos de golpe del raycasting.
- PL_RAY_NORMAL_RESULT obtener las normales del raycasting.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si se intenta obtener el parametro PL_RAY_POINT_RESULT o PL_RAY_NORMAL_RESULT sin antes haber hecho una prueba de raycasting. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |
| `PL_INVALID_VALUE` | si lenght es 0 o incorrecto. |

## `plContactCallBack`

Poner el listener de contactos

```c
void plContactCallBack(obtContactCallBack callback);
```

### Parámetros

| Nombre | |
| --- | --- |
| `callback` | poner funcion callback. |

### Notas

{: .note }
- Para que funcione debes poner antes de esta funcion plDynamicWorldi(PL_CONTACT_TEST,1)
- Para crear la funcion callback requerida.
- bool name_function(int user_ptr0,bool match0,int user_ptr1,bool match1){}

</div>

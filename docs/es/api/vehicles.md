---
title: Vehículos (raycast)
parent: Referencia API
grand_parent: Inicio
nav_order: 7
---

# Vehículos (raycast)

<div class="openpl-api" markdown="1">

## `plGenVehicle`

Generar un vehiculo.

```c
Pluint plGenVehicle();
```

### Notas

{: .note }
- Esta funcion retorna una id y crea una cola de variables necesarias antes de crear una instancia del objeto.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si hay una cola pendiente para crear un vehiculo. |

## `plBindVehicle`

Enlazar para usar un vehiculo. Para la simulacion de suspension.

```c
void plBindVehicle(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `veh` | La id del vehiculo a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar un vehiculo, asegurate de que no haya sido enlazado previamente, si es asi para desenlazar veh debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si ya hay un vehiculo enlazado previamente. |

## `plDeleteVehicle`

Eliminar un vehiculo.

```c
void plDeleteVehicle(Pluint indx);
```

### Parámetros

| Nombre | |
| --- | --- |
| `veh` | La id del vehiculo a eliminar. |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el vehiculo esta enlazado o esta siendo usado. Tambien puede ser causado por que el vehiculo que tratas de eliminar ya no existe. |

## `plWheelf`

Poner un valor a un parametro especifico

```c
void plWheelf(Plint wheel, Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_WHEEL_ENGINE_FORCE, PL_WHEEL_STEERING, PL_WHEEL_BRAKE, PL_WHEEL_ROLL_INFLUENCE, |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_WHEEL_ENGINE_FORCE poner masa en kilogramos. value debe ser la masa en kilogramos.
- PL_WHEEL_STEERING factor angular. value debe ser el factor angular.
- PL_WHEEL_BRAKE friccion de rodadura. value debe ser el valor equivalente
- PL_WHEEL_ROLL_INFLUENCE friccion giratoria. value debe ser el valor equivalente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No se producen errores. |

## `plVehiclei`

Poner un valor a un parametro especifico

```c
void plVehiclei(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_VEHICLE_CHASSIS. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_VEHICLE_CHASSIS poner el id del cuerpo rigido que sera el chasis. value no debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No se producen errores. |

## `plVehiclef`

Poner un valor a un parametro especifico

```c
void plVehiclef(Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_SUSPENSION_STIFFNESS, PL_SUSPENSION_DAMPING, PL_SUSPENSION_COMPRESION, PL_MAX_SUSPENSION_TRAVEL, PL_FRICTION_SLIP, PL_MAX_SUSPENSION_FORCE. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_SUSPENSION_STIFFNESS poner masa en kilogramos. value debe ser la masa en kilogramos.
- PL_SUSPENSION_DAMPING factor angular. value debe ser el factor angular.
- PL_SUSPENSION_COMPRESION friccion de rodadura. value debe ser el valor equivalente
- PL_MAX_SUSPENSION_TRAVEL friccion giratoria. value debe ser el valor equivalente.
- PL_FRICTION_SLIP friccion. value debe ser el valor equivalente.
- PL_MAX_SUSPENSION_FORCE coeficiente de restitucion. value debe ser el valor equivalente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No se producen errores. |

## `plVehicle3f`

```c
void plVehicle3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Plenum |
| `x` | Plfloat |
| `y` | Plfloat |
| `z` | Plfloat |

## `plVehiclefv`

Poner un valor a un parametro especifico

```c
void plVehiclefv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_ADD_WHEEL. |
| `values` | Los valores asignados. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_ADD_WHEEL agregar informacion de una rueda. El arreglo debe tener la siguiente estructura de datos:
- lenght debe ser 12

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plGetWheelfv`

Obtener un valor a un parametro especifico

```c
void plGetWheelfv(Plint wheel, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `wheel` | El index de la llanta |
| `param` | Especifica el parametro para obtener el valor. Puede ser PL_WHEEL_TRANSFORM, PL_WHEEL_TRANSFORM_INTERPOLATION. |
| `values` | Valores para obtener. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_WHEEL_TRANSFORM obtener la transformacion de la llanta. values es un matrix4x4. lenght debe ser 16
- PL_WHEEL_TRANSFORM_INTERPOLATION obtener la transformacion de la llanta. (interpolado)* values es un matrix4x4. lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_OPERATION` | en raras ocaciones. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

</div>

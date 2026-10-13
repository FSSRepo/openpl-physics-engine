---
title: Núcleo
parent: Referencia API
grand_parent: Inicio
nav_order: 1
---

# Núcleo

<div class="openpl-api" markdown="1">

## `plCreateContext`

Crea el contexto para poder utilizar la API.

```c
PLbool plCreateContext();
```

### Notas

{: .note }
- Esta funcion retorna 1 si el contexto fue creado o si ya se creo anterormente.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | Esta funcion retorna 0 si ocurrio un problema al crear el contexto. |

## `plDestroyContext`

Destruye el contexto y libera la memoria.

```c
void plDestroyContext();
```

### Notas

{: .note }
- Cuando ya no necesites utilizar la API o al cerrar la aplicacion no olvides destruir el contexto y liberar la memoria.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No suelen suceder errores. |

## `plSetNumThreads`

```c
PLbool plSetNumThreads(Plint numThreads);
```

### Parámetros

| Nombre | |
| --- | --- |
| `numThreads` | Plint |

## `plGetError`

```c
Plint plGetError();
```

## `plGetString`

Obtener el valor especificado por los parametros.

```c
const char* plGetString(Plenum param);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro a obtener. Puede ser PL_VERSION, PL_VBASED, PL_DEBUG_INFO, PL_VENDOR |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_VERSION retorna la version de OpenPL.
- PL_VBASED retorna la version de bullet physics en la que se basa.
- PL_VENDOR retorna el nombre de la empresa encargada del proyecto.
- PL_DEBUG_INFO retorna la informacion recopilada si el modo depuracion esta activado.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si el paramero especificado no es aceptado. |

## `plGetInteger`

Obtener el valor especificado por los parametros.

```c
Plint plGetInteger(Plenum param);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro a obtener. Puede ser PL_VERSION o PL_VBASED |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_VERSION retorna la version de OpenPL.
- PL_VBASED retorna la version de bullet physics en la que se basa.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_ENUM` | si el paramero especificado no es aceptado. |

## `plCreate`

Crea objetos tales como: cuerpos rigidos, restricciones, vehiculos, formas de colision, personajes.

```c
void plCreate(Plenum target);
```

### Parámetros

| Nombre | |
| --- | --- |
| `target` | Especifica el tipo de objeto que deseas crear. Puede ser PL_RIGID_BODY, PL_TYPED_CONSTRAINT, PL_RAYCAST_VEHICLE, PL_CHARACTER, ademas de tipos de collision. |

### Notas

{: .note }
- Antes de llamar a esta funcion debes tener todos los valores necesarios ya predefinidos.
- Las siguientes constantes son aceptadas por param:
- PL_RIGID_BODY crear un cuerpo rigido.
- PL_TYPED_CONSTRAINT crear una restriccion.
- PL_RAYCAST_VEHICLE crear un vehiculo.
- PL_CHARACTER crear un personaje.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si intentas crear un cuerpo rigido sin enlazar la id. |
| `PL_INVALID_VALUE` | hacen faltan algunos valores para crear el objeto. |
| `PL_INVALID_OPERATION` | si no has generado el objeto que quieres crear. |

## `plStepSimulation`

Actualizar la simulacion.

```c
void plStepSimulation(Plfloat timeStep, Plsizei maxSubSteps, Plfloat fixedTimeStep);
```

### Parámetros

| Nombre | |
| --- | --- |
| `timeStep` | El tiempo de cada fotograma |
| `maxSubSteps` | limite de fotogramas por segundo |
| `fixedTimeStep` | Tiempo correcto por cada fotograma |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No suelen suceder errores. |

</div>

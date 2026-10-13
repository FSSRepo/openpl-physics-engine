---
title: Core
parent: API Reference
grand_parent: Home
nav_order: 1
---

# Core

<div class="openpl-api" markdown="1">

## `plCreateContext`

Creates the context required to use the API.

```c
PLbool plCreateContext();
```

### Notes

{: .note }
- This function returns 1 if the context was created or was already created previously.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | This function returns 0 if a problem occurred while creating the context. |

## `plDestroyContext`

Destroys the context and frees the memory.

```c
void plDestroyContext();
```

### Notes

{: .note }
- When you no longer need the API, or when closing the application, do not forget to destroy the context and free the memory.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | Errors usually do not occur. |

## `plSetNumThreads`

```c
PLbool plSetNumThreads(Plint numThreads);
```

### Parameters

| Name | |
| --- | --- |
| `numThreads` | Plint |

## `plGetError`

```c
Plint plGetError();
```

## `plGetString`

Get the value specified by the parameters.

```c
const char* plGetString(Plenum param);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get. Can be PL_VERSION, PL_VBASED, PL_DEBUG_INFO, PL_VENDOR. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_VERSION returns the OpenPL version.
- PL_VBASED returns the version of Bullet Physics it is based on.
- PL_VENDOR returns the name of the company in charge of the project.
- PL_DEBUG_INFO returns the information gathered if debug mode is enabled.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if the specified parameter is not accepted. |

## `plGetInteger`

Get the value specified by the parameters.

```c
Plint plGetInteger(Plenum param);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get. Can be PL_VERSION or PL_VBASED. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_VERSION returns the OpenPL version.
- PL_VBASED returns the version of Bullet Physics it is based on.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if the specified parameter is not accepted. |

## `plCreate`

Creates objects such as: rigid bodies, constraints, vehicles, collision shapes, characters.

```c
void plCreate(Plenum target);
```

### Parameters

| Name | |
| --- | --- |
| `target` | Specifies the type of object you want to create. Can be PL_RIGID_BODY, PL_TYPED_CONSTRAINT, PL_RAYCAST_VEHICLE, PL_CHARACTER, in addition to collision types. |

### Notes

{: .note }
- Before calling this function you must already have all the required values predefined.
- The following constants are accepted by param:
- PL_RIGID_BODY creates a rigid body.
- PL_TYPED_CONSTRAINT creates a constraint.
- PL_RAYCAST_VEHICLE creates a vehicle.
- PL_CHARACTER creates a character.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if you try to create a rigid body without binding the id. |
| `PL_INVALID_VALUE` | some values are missing to create the object. |
| `PL_INVALID_OPERATION` | if you have not generated the object you want to create. |

## `plStepSimulation`

Update the simulation.

```c
void plStepSimulation(Plfloat timeStep, Plsizei maxSubSteps, Plfloat fixedTimeStep);
```

### Parameters

| Name | |
| --- | --- |
| `timeStep` | The time of each frame |
| `maxSubSteps` | frame per second limit |
| `fixedTimeStep` | Correct time per frame |

### Errors

| Code | When it happens |
| --- | --- |
| `—` | Errors usually do not occur. |

</div>

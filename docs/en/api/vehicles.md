---
title: Vehicles (raycast)
parent: API Reference
grand_parent: Home
nav_order: 7
---

# Vehicles (raycast)

<div class="openpl-api" markdown="1">

## `plGenVehicle`

Generate a vehicle.

```c
Pluint plGenVehicle();
```

### Notes

{: .note }
- This function returns an id and creates a queue of the variables needed before creating an instance of the object.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if there is a pending queue to create a vehicle. |

## `plBindVehicle`

Bind to use a vehicle. For suspension simulation.

```c
void plBindVehicle(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `veh` | The id of the vehicle to bind. |

### Notes

{: .note }
- Before calling this function to bind a vehicle, make sure it was not bound previously; if so, to unbind, veh must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if a vehicle is already bound previously. |

## `plDeleteVehicle`

Delete a vehicle.

```c
void plDeleteVehicle(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `veh` | The id of the vehicle to delete. |

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the vehicle is bound or is being used. It can also be caused by the vehicle you are trying to delete no longer existing. |

## `plWheelf`

Set a value for a specific parameter

```c
void plWheelf(Plint wheel, Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_WHEEL_ENGINE_FORCE, PL_WHEEL_STEERING, PL_WHEEL_BRAKE, PL_WHEEL_ROLL_INFLUENCE, |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_WHEEL_ENGINE_FORCE sets mass in kilograms. value must be the mass in kilograms.
- PL_WHEEL_STEERING angular factor. value must be the angular factor.
- PL_WHEEL_BRAKE rolling friction. value must be the equivalent value
- PL_WHEEL_ROLL_INFLUENCE spinning friction. value must be the equivalent value.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | No errors occur. |

## `plVehiclei`

Set a value for a specific parameter

```c
void plVehiclei(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_VEHICLE_CHASSIS. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_VEHICLE_CHASSIS sets the id of the rigid body that will be the chassis. value must not be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | No errors occur. |

## `plVehiclef`

Set a value for a specific parameter

```c
void plVehiclef(Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_SUSPENSION_STIFFNESS, PL_SUSPENSION_DAMPING, PL_SUSPENSION_COMPRESION, PL_MAX_SUSPENSION_TRAVEL, PL_FRICTION_SLIP, PL_MAX_SUSPENSION_FORCE. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SUSPENSION_STIFFNESS sets mass in kilograms. value must be the mass in kilograms.
- PL_SUSPENSION_DAMPING angular factor. value must be the angular factor.
- PL_SUSPENSION_COMPRESION rolling friction. value must be the equivalent value
- PL_MAX_SUSPENSION_TRAVEL spinning friction. value must be the equivalent value.
- PL_FRICTION_SLIP friction. value must be the equivalent value.
- PL_MAX_SUSPENSION_FORCE restitution coefficient. value must be the equivalent value.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | No errors occur. |

## `plVehicle3f`

```c
void plVehicle3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Plenum |
| `x` | Plfloat |
| `y` | Plfloat |
| `z` | Plfloat |

## `plVehiclefv`

Set a value for a specific parameter

```c
void plVehiclefv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_ADD_WHEEL. |
| `values` | The assigned values. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_ADD_WHEEL adds information about a wheel. The array must have the following data structure:
- lenght must be 12

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetWheelfv`

Get a value for a specific parameter

```c
void plGetWheelfv(Plint wheel, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `wheel` | The wheel index |
| `param` | Specifies the parameter to get the value. Can be PL_WHEEL_TRANSFORM, PL_WHEEL_TRANSFORM_INTERPOLATION. |
| `values` | Values to get. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_WHEEL_TRANSFORM gets the wheel transformation. values is a matrix4x4. lenght must be 16
- PL_WHEEL_TRANSFORM_INTERPOLATION gets the wheel transformation. (interpolated)* values is a matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

</div>

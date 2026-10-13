---
title: Dynamic world
parent: API Reference
grand_parent: Home
nav_order: 2
---

# Dynamic world

<div class="openpl-api" markdown="1">

## `plDynamicWorldi`

Set a value for a specific parameter

```c
void plDynamicWorldi(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_ADD_RIGID_BODY, PL_REMOVE_RIGID_BODY, PL_ADD_CONSTRAINT, PL_REMOVE_CONSTRAINT, PL_INIT_VEHICLE_RAYCASTER, PL_INIT_CHARACTER, PL_ADD_CHARACTER, PL_CLEAR_RAY_RESULTS, PL_USE_DBVT, PL_CONTACT_TEST, PL_ADD_FLAG_RAY_TEST, PL_USER_POINTER, PL_DEBUG_MODE. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_ADD_RIGID_BODY adds a rigid body. value must be the id of the already created rigid body.
- PL_REMOVE_RIGID_BODY removes a rigid body. value must be the id of the rigid body to remove.
- PL_ADD_CONSTRAINT adds a constraint. value must be the id of the already created constraint.
- PL_REMOVE_CONSTRAINT removes a constraint. value must be the id of the constraint to remove.
- PL_INIT_VEHICLE_RAYCASTER initializes the vehicle ray caster. value must be 0.
- PL_CLEAR_RAY_RESULTS clears the ray test results. value must be 0.
- PL_USE_DBVT uses the Dbvt broadphase. value must be 0.
- PL_CONTACT_TEST value must be 1 if you want to use the contact test or 0 if not.
- PL_ADD_FLAG_RAY_TEST adds a filter to the ray casting test. value can be PL_RF_FLT_BACK_FACES, PL_RF_KP_UNFLIPPED_NORM, PL_RF_SS_CCR, PL_RF_GJK_CCR.
- PL_USER_POINTER finds and binds the rigid body from the user pointer. value must be the user pointer.
- PL_DEBUG_MODE value must be 1 if you want to use debug mode or 0 if not.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body, constraint or character do not exist or the id is incorrect. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plDynamicWorld3f`

Set a value for a specific parameter

```c
void plDynamicWorld3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_GRAVITY, PL_USE_AXIS_SWEEP3. |
| `v0 v1 v2` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_GRAVITY sets the gravity vector, by default (0,-9.8,0). v0 v1 v2 must be the gravity vector (x,y,x).
- PL_USE_AXIS_SWEEP3 uses AxisSweep3. v0 v1 v2 must be the extent vector.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if the specified parameter is not allowed. |

## `plDynamicWorldfv`

Set a value for a specific parameter

```c
void plDynamicWorldfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_GRAVITY, PL_RAY_CLOSEST_TEST, PL_RAY_ALL_TEST, PL_USE_AXIS_SWEEP3. |
| `values` | The array of values |
| `lenght` | Length of the array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_GRAVITY sets the gravity vector. values must be the vector (x,y,z).
- PL_RAY_CLOSEST_TEST does a ray casting test, the closest object. values must be the ray origin vector (x,y,z) and ray end (x,y,z).
- PL_RAY_ALL_TEST does a ray casting test, all objects without exception. values must be the ray origin vector (x,y,z) and ray end (x,y,z).
- PL_USE_AXIS_SWEEP3 uses AxisSweep3 broadphase. values must be the min and max vector.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_VALUE` | if the lenght value is 0 or is not correct. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetDynamicWorldi`

Get a value through a specific parameter.

```c
void plGetDynamicWorldi(Plenum param, Plint* value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_NUM_RIGID_BODY, PL_RAY_BODY_RESULT, PL_NUM_RAY_RESULT. |
| `value` | The value to get. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_NUM_RIGID_BODY gets the number of rigid bodies.
- PL_RAY_BODY_RESULT gets the rigid body that was hit by the raycasting.
- PL_NUM_RAY_RESULT gets the number of objects hit by the raycasting.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetDynamicWorld3f`

Get a value through a specific parameter.

```c
void plGetDynamicWorld3f(Plenum param, Plfloat* v1, Plfloat* v2, Plfloat* v3);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the values. Can be PL_GRAVITY, PL_RAY_POINT_RESULT. |
| `v0 v1 v2` | The values to get. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_GRAVITY gets the gravity vector in the order (x,y,z).
- PL_RAY_POINT_RESULT gets the hit point of the ray casting (x,y,z).

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetDynamicWorldiv`

Get a value through a specific parameter.

```c
void plGetDynamicWorldiv(Plenum param, Plint* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the values. Can be PL_RAY_BODY_RESULT. |
| `values` | The array of values |
| `lenght` | Length of the array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_RAY_BODY_RESULT gets the ids of the rigid bodies that were hit by the ray casting.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if you try to get the PL_RAY_BODY_RESULT parameter without having done a raycasting test before. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |
| `PL_INVALID_VALUE` | if lenght is 0 or incorrect. |

## `plGetDynamicWorldfv`

Get a value through a specific parameter.

```c
void plGetDynamicWorldfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_RAY_POINT_RESULT, PL_RAY_NORMAL_RESULT. |
| `values` | The array of values |
| `lenght` | Length of the array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_RAY_POINT_RESULT gets the hit points of the raycasting.
- PL_RAY_NORMAL_RESULT gets the normals of the raycasting.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if you try to get the PL_RAY_POINT_RESULT or PL_RAY_NORMAL_RESULT parameter without having done a raycasting test before. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |
| `PL_INVALID_VALUE` | if lenght is 0 or incorrect. |

## `plContactCallBack`

Set the contact listener

```c
void plContactCallBack(obtContactCallBack callback);
```

### Parameters

| Name | |
| --- | --- |
| `callback` | set callback function. |

### Notes

{: .note }
- For it to work you must call plDynamicWorldi(PL_CONTACT_TEST,1) before this function
- To create the required callback function.
- bool name_function(int user_ptr0,bool match0,int user_ptr1,bool match1){}

</div>

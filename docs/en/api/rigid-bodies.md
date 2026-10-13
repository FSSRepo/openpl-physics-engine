---
title: Rigid bodies
parent: API Reference
grand_parent: Home
nav_order: 4
---

# Rigid bodies

<div class="openpl-api" markdown="1">

## `plRigidBodyi`

Set a value for a specific parameter

```c
void plRigidBodyi(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_USER_POINTER, PL_COLLISION_CALLBACK_FILTER, PL_COLLISION_CALLBACK_FLAG, PL_USER_INDEX, PL_ACTIVATION_STATE, PL_ADD_COLLISION_FLAG, PL_COLLISION_SHAPE, PL_ANISOTROPIC_ROLLING_FRICTION. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_USER_POINTER sets a user pointer. value must be the user pointer you want.
- PL_COLLISION_CALLBACK_FILTER sets the filter for the contact test. value must be the filter.
- PL_COLLISION_CALLBACK_FLAG assigns a feature to filter in the contact test. value must be the feature
- PL_USER_INDEX sets a user index. value must be the index.
- PL_ACTIVATION_STATE changes the activation state. value must be PL_DIS_DACTIVATION if you want the rigid body to remain active all the time, PL_DIS_SIMULATION if you want the rigid body to be ignored during the simulation, PL_ACTIVATE if you want to activate the rigid body.
- PL_ADD_COLLISION_FLAG assigns collision features. value if you want the rigid body, PL_CF_KINEMATIC_OBJECT kinematic, PL_CF_STATIC_OBJECT static, PL_CF_CUSTOM_MATERIAL_CALLBACK custom materials.
- PL_COLLISION_SHAPE uses Dbvt broadphase. value must be 0.
- PL_ANISOTROPIC_ROLLING_FRICTION sets the anisotropic friction. value must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | if you try to use PL_COLLISION_CALLBACK_FILTER, PL_COLLISION_CALLBACK_FLAG without first enabling the contact test or without setting the user pointer. |

## `plRigidBodyf`

Set a value for a specific parameter

```c
void plRigidBodyf(Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_MASS, PL_ANGULAR_FACTOR, PL_ROLLING_FRICTION, PL_SPINNING_FRICTION, PL_FRICTION, PL_RESTITUTION. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_MASS sets mass in kilograms. value must be the mass in kilograms.
- PL_ANGULAR_FACTOR angular factor. value must be the angular factor.
- PL_ROLLING_FRICTION rolling friction. value must be the equivalent value
- PL_SPINNING_FRICTION spinning friction. value must be the equivalent value.
- PL_FRICTION friction. value must be the equivalent value.
- PL_RESTITUTION restitution coefficient. value must be the equivalent value.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plRigidBody3f`

Set a value for a specific parameter

```c
void plRigidBody3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_POSITION, PL_MOTION_STATE_POSITION, PL_DAMPING, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_APPLY_CENTRAL_FORCE PL_APPLY_TORQUE, PL_APPLY_CENTRAL_IMPULSE, PL_APPLY_TORQUE_IMPULSE, PL_ANISOTROPIC_FRICTION, PL_COL_INTERP_LINEAR_VEL, PL_COL_INTERP_ANGULAR_VEL. |
| `x y z` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_POSITION sets the center-of-mass position. x y z must be the position vector (x,y,z).
- PL_MOTION_STATE_POSITION sets the motion-state position. x y z must be the position vector (x,y,z).
- PL_DAMPING sets the linear and angular damping factor. x must be the linear damping factor. y must be the angular damping factor.
- PL_LINEAR_FACTOR sets the linear factor. x y z is the vector (x,y,z).
- PL_ANGULAR_FACTOR sets the spinning factor. x y z is the vector (x,y,z).
- PL_LOCAL_INERTIA sets the local inertia axis. x y z must be the inertia vector (x,y,z).
- PL_APPLY_CENTRAL_FORCE applies a central force. x y z is the force vector to apply (x,y,z).
- PL_APPLY_TORQUE applies torque. x y z is the torque vector (x,y,z).
- PL_APPLY_CENTRAL_IMPULSE applies a central impulse. x y z must be the impulse vector (x,y,z).
- PL_APPLY_TORQUE_IMPULSE applies impulse with torque. x y z is the impulse vector (x,y,z).
- PL_ANISOTROPIC_FRICTION sets the anisotropic friction factor. x y z is the vector (x,y,z).
- PL_COL_INTERP_LINEAR_VEL sets the interpolated linear velocity. x y z is the vector (x,y,z).
- PL_COL_INTERP_ANGULAR_VEL sets the interpolated spinning velocity. x y z is the vector (x,y,z).

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plRigidBodyfv`

Set a value for a specific parameter

```c
void plRigidBodyfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_POSITION, PL_MOTION_STATE_POSITION, PL_BASIS, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_APPLY_CENTRAL_FORCE PL_APPLY_TORQUE, PL_APPLY_CENTRAL_IMPULSE, PL_APPLY_TORQUE_IMPULSE, PL_ORIENTATION, PL_TRANSFORM, PL_MOTION_STATE_TRANSFORM, PL_COL_WORLD_TRANSFORM, PL_COL_INTERP_WORLD_TRANSFORM. |
| `values` | The assigned values. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_POSITION sets the center-of-mass position. values must be the position vector. lenght must be 3
- PL_MOTION_STATE_POSITION sets the motion-state position. values must be the position vector (x,y,z). lenght must be 3
- PL_BASIS sets a new basis. values must be a matrix3x3. lenght must be 3
- PL_LINEAR_FACTOR sets the linear factor. values is the vector. lenght must be 3
- PL_ANGULAR_FACTOR sets the spinning factor. values is the vector. lenght must be 3
- PL_LOCAL_INERTIA sets the local inertia axis. values must be the inertia vector. lenght must be 3
- PL_APPLY_CENTRAL_FORCE applies a central force. values is the force vector to apply. lenght must be 3
- PL_APPLY_TORQUE applies torque. values is the torque vector. lenght must be 3
- PL_APPLY_CENTRAL_IMPULSE applies a central impulse. values must be the impulse vector. lenght must be 3
- PL_APPLY_TORQUE_IMPULSE applies impulse with torque. values is the impulse vector. lenght must be 3
- PL_ORIENTATION sets the orientation or rotation. values is the rotation quaternion. lenght must be 4
- PL_TRANSFORM sets the transformation. values must be a transformation matrix4x4. lenght must be 16
- PL_MOTION_STATE_TRANSFORM sets the transformation in the motion state. values must be a transformation matrix4x4. lenght must be 16
- PL_COL_WORLD_TRANSFORM sets the world transformation. values must be a transformation matrix4x4. lenght must be 16
- PL_COL_INTERP_WORLD_TRANSFORM sets the interpolated world transformation. values must be a transformation matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetRigidBodyi`

Get a value by a specific parameter

```c
void plGetRigidBodyi(Pluint body, Plenum param, Plint* value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_USER_POINTER, PL_USER_INDEX, PL_IS_IN_WORLD. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_USER_POINTER gets the user pointer.
- PL_USER_INDEX gets the user index.
- PL_IS_IN_WORLD gets whether the rigid body is in the dynamic world or not.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetRigidBodyf`

Get a value for a specific parameter

```c
void plGetRigidBodyf(Pluint body, Plenum param, Plfloat* value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_MASS, PL_ANGULAR_FACTOR, PL_ROLLING_FRICTION, PL_SPINNING_FRICTION, PL_FRICTION, PL_RESTITUTION. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_MASS gets the mass in kilograms. value must be the mass in kilograms.
- PL_ANGULAR_FACTOR angular factor. value must be the angular factor.
- PL_ROLLING_FRICTION rolling friction. value must be the equivalent value
- PL_SPINNING_FRICTION spinning friction. value must be the equivalent value.
- PL_FRICTION friction. value must be the equivalent value.
- PL_RESTITUTION restitution coefficient. value must be the equivalent value.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetRigidBody3f`

Get a value for a specific parameter

```c
void plGetRigidBody3f(Pluint body, Plenum param, Plfloat* v1, Plfloat* v2, Plfloat* v3);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_POSITION, PL_DAMPING, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_TOTAL_FORCE PL_TOTAL_TORQUE, PL_LINEAR_VELOCITY, PL_ANGULAR_VELOCITY. |
| `v1 v2 v3` | The obtained values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_POSITION gets the center-of-mass position. v1 v2 v3 is the position vector (x,y,z).
- PL_DAMPING gets the linear and angular damping factor. v1 is the linear damping value and v2 is the angular damping value.
- PL_LINEAR_FACTOR gets the linear factor. v1 v2 v3 is the vector (x,y,z).
- PL_ANGULAR_FACTOR gets the spinning factor. v1 v2 v3 is the vector (x,y,z).
- PL_LOCAL_INERTIA gets the local inertia axis. v1 v2 v3 is the vector (x,y,z).
- PL_TOTAL_FORCE gets the total forces. v1 v2 v3 is the vector (x,y,z).
- PL_TOTAL_TORQUE gets the total torque. v1 v2 v3 is the vector (x,y,z).
- PL_LINEAR_VELOCITY gets the linear velocity. v1 v2 v3 is the vector (x,y,z).
- PL_ANGULAR_VELOCITY gets the angular velocity. v1 v2 v3 is the vector (x,y,z).

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetRigidBodyfv`

Get a value for a specific parameter

```c
void plGetRigidBodyfv(Pluint body, Plenum param, Plfloat* values, Plint lenght);
```

### Parameters

| Name | |
| --- | --- |
| `body` | The id of the rigid body |
| `param` | Specifies the parameter to get the value. Can be PL_POSITION, PL_MOTION_STATE_POSITION, PL_BASIS, PL_LINEAR_FACTOR, PL_ANGULAR_FACTOR, PL_LOCAL_INERTIA, PL_TOTAL_TORQUE PL_TOTAL_FORCE, PL_LINEAR_VELOCITY, PL_ANGULAR_VELOCITY, PL_ORIENTATION, PL_TRANSFORM, PL_MOTION_STATE_TRANSFORM. |
| `values` | Values to get. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_POSITION gets the center-of-mass position. values is a vector3. lenght must be 3
- PL_MOTION_STATE_POSITION gets the motion-state position. values is a vector3. lenght must be 3
- PL_BASIS gets the basis of the center-of-mass transformation. values must be a matrix3x3. lenght must be 3
- PL_LINEAR_FACTOR gets the linear factor. values is a vector3. lenght must be 3
- PL_ANGULAR_FACTOR gets the spinning factor. values is a vector3. lenght must be 3
- PL_LOCAL_INERTIA gets the local inertia axis. values is a vector3. lenght must be 3
- PL_TOTAL_FORCE gets the total force. values is a vector3. lenght must be 3
- PL_TOTAL_TORQUE gets the total torque. values is a vector3. lenght must be 3
- PL_LINEAR_VELOCITY gets the linear velocity. values is a vector3. lenght must be 3
- PL_ANGULAR_VELOCITY gets the angular or spinning velocity. values is a vector3. lenght must be 3
- PL_ORIENTATION sets the orientation or rotation. values is the rotation quaternion. lenght must be 4
- PL_TRANSFORM sets the transformation. values must be a transformation matrix4x4. lenght must be 16
- PL_MOTION_STATE_TRANSFORM sets the transformation in the motion state. values must be a transformation matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

</div>

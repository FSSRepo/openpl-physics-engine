---
title: Soft bodies
parent: API Reference
grand_parent: Home
nav_order: 9
---

# Soft bodies

<div class="openpl-api" markdown="1">

## `plGenSoftBody`

Generate a soft body and enqueue it.

```c
Pluint plGenSoftBody(Plenum type);
```

### Parameters

| Name | |
| --- | --- |
| `type` | Specifies the type of soft body to create. Can be PL_SOFTBODY_ROPE, PL_SOFTBODY_CLOTH, PL_SOFTBODY_ELLIPSOID, PL_SOFTBODY_MESH. |

### Notes

{: .note }
- This function returns an id and creates a queue of the variables needed before creating an instance of the object.
- The PL_SOFTBODY_MESH type requires the buffers to be defined beforehand with plBufferData.
- To materialize the soft body you must call plCreate(PL_SOFT_BODY).

### Errors

| Code | When it happens |
| --- | --- |
| `—` | This function returns 0 if there is no context or there is a global error. |

## `plBindSoftBody`

Bind to use a soft body.

```c
void plBindSoftBody(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | The id of the soft body to bind. |

### Notes

{: .note }
- Before calling this function to bind a soft body, make sure it was not bound previously; if so, to unbind, indx must be 0.

## `plDeleteSoftBody`

Delete a soft body.

```c
void plDeleteSoftBody(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | The id of the soft body to delete. |

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body was already deleted or does not exist. |

## `plSoftBodyi`

Set a value for a specific parameter

```c
void plSoftBodyi(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_SB_RESOLUTION, PL_SB_FIXED_NODES, PL_SB_ITERATIONS, PL_SB_ANCHOR_BODY, PL_SB_AERO_MODEL, PL_SB_RANDOMIZE_CONSTRAINTS. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_RESOLUTION sets the node resolution on X when generating the body. Only valid in the queue.
- PL_SB_FIXED_NODES number of fixed nodes. Only valid in the queue.
- PL_SB_ITERATIONS solver iterations (piterations and viterations).
- PL_SB_ANCHOR_BODY anchors the soft body to a rigid body; value must be the id of the rigid body.
- PL_SB_AERO_MODEL aerodynamic model (value of the btSoftBody::eAeroModel enum).
- PL_SB_RANDOMIZE_CONSTRAINTS value must be 1 (PL_TRUE) to randomize the constraints or 0 if not. Only valid in the queue.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body does not exist or the id is incorrect. |

## `plSoftBodyf`

Set a value for a specific parameter

```c
void plSoftBodyf(Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_SB_RADIUS, PL_SB_TOTAL_MASS, PL_SB_PRESSURE, PL_SB_STIFFNESS, PL_SB_DAMPING, PL_SB_VELOCITY_CORRECTION. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_RADIUS radius of the collision sphere of each node. Only valid in the queue.
- PL_SB_TOTAL_MASS total mass of the soft body.
- PL_SB_PRESSURE internal pressure (kPR).
- PL_SB_STIFFNESS material stiffness (kLST, kAST and kVST).
- PL_SB_DAMPING damping (kDP).
- PL_SB_VELOCITY_CORRECTION velocity correction factor (kVCF).

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body does not exist or the id is incorrect. |

## `plSoftBody3f`

Set a value for a specific parameter

```c
void plSoftBody3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_SB_POSITION_A, PL_SB_POSITION_B, PL_SB_POSITION_C, PL_SB_POSITION_D. |
| `x y z` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_POSITION_A first point (x,y,z) that defines the position of the soft body.
- PL_SB_POSITION_B second point (x,y,z).
- PL_SB_POSITION_C third point (x,y,z).
- PL_SB_POSITION_D fourth point (x,y,z).
- These parameters are only valid during the queue (before plCreate).

## `plSoftBodyfv`

Set a value for a specific parameter

```c
void plSoftBodyfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_SB_RESOLUTION, PL_SB_ANCHOR_NODES. |
| `values` | The array of values. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_RESOLUTION sets the X and Y resolution; values[0] is X and values[1] is Y, lenght must be 2 or more.
- PL_SB_ANCHOR_NODES indices of the anchored nodes; values holds the indices and lenght the count.

## `plGetSoftBodyi`

Get an integer value through a specific parameter.

```c
void plGetSoftBodyi(Pluint indx, Plenum param, Plint* value);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Soft body ID. |
| `param` | Specifies the parameter to get the value. Can be PL_SB_NUM_NODES. |
| `value` | The obtained value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_NUM_NODES gets the number of nodes of the soft body.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body does not exist or the id is incorrect. |

## `plGetSoftBodyf`

Get a floating-point value through a specific parameter.

```c
void plGetSoftBodyf(Pluint indx, Plenum param, Plfloat* value);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Soft body ID. |
| `param` | Specifies the parameter to get the value. Can be PL_SB_VOLUME, PL_SB_TOTAL_MASS. |
| `value` | The obtained value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_VOLUME gets the volume of the soft body.
- PL_SB_TOTAL_MASS gets the total mass of the soft body.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body does not exist or the id is incorrect. |

## `plGetSoftBodyfv`

Get an array of values through a specific parameter.

```c
void plGetSoftBodyfv(Pluint indx, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Soft body ID. |
| `param` | Specifies the parameter to get the values. Can be PL_SB_NODE_POSITION. |
| `values` | Values to get. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SB_NODE_POSITION gets the (x,y,z) positions of all nodes; values must have room for num_nodes * 3 and lenght must be num_nodes * 3 or more.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the soft body does not exist or the id is incorrect. |
| `PL_INVALID_VALUE` | if the output array is too small. |

</div>

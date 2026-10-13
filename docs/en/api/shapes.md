---
title: Collision shapes
parent: API Reference
grand_parent: Home
nav_order: 5
---

# Collision shapes

<div class="openpl-api" markdown="1">

## `plGenShape`

```c
Pluint plGenShape();
```

## `plBindShape`

Bind to use a collision shape.

```c
void plBindShape(Pluint shape);
```

### Parameters

| Name | |
| --- | --- |
| `shape` | The id of the collision shape to bind. |

### Notes

{: .note }
- Before calling this function to bind a collision shape, make sure it was not bound previously; if so, to unbind, shape must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if a collision shape is already bound previously. |

## `plShapei`

Set a value for a specific parameter

```c
void plShapei(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_USER_POINTER, PL_ADD_CHILD_SHAPE, PL_REMOVE_CHILD_SHAPE, PL_OPTIMIZE_CONVEX_HULL. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_USER_POINTER sets a user pointer. value must be the user pointer you want.
- PL_ADD_CHILD_SHAPE adds a collision shape to a compound. value must be the collision shape to add.
- PL_REMOVE_CHILD_SHAPE removes a collision shape from a compound. value must be the collision shape to remove.
- PL_OPTIMIZE_CONVEX_HULL optimizes the convex hull shape. value must be 1 or 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if param is not allowed. |

## `plShapef`

Set a value for a specific parameter

```c
void plShapef(Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_HEIGHT, PL_RADIUS, PL_PLANE_CONSTANT. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_HEIGHT sets the height of a cone, capsule or cylinder. value must never be 0.
- PL_RADIUS sets the radius of a sphere or of the ends of a cylinder, capsule or cone. value must never be 0.
- PL_PLANE_CONSTANT sets the height of a static plane. value must be the equivalent value.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if you try to assign one of these values after creating the collision shape. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plShape3f`

Set a value for a specific parameter

```c
void plShape3f(Plenum param, Plfloat v1, Plfloat v2, Plfloat v3);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_EXTENT, PL_PLANE_NORMAL, PL_LOCAL_SCALING. |
| `x y z` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_EXTENT sets the extent. x y z must be the vector (x,y,z).
- PL_PLANE_NORMAL sets the collision direction of a static plane. x y z must be the normal vector (x,y,z).
- PL_LOCAL_SCALING scales a collision shape. x y z must be the scale vector (x,y,z).

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plShapefv`

Set a value for a specific parameter

```c
void plShapefv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_TRANSFORM. |
| `values` | The assigned values. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_TRANSFORM sets the transformation for the collision shape. values must be a transformation matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plBufferData`

Set a data buffer.

```c
void plBufferData(Plenum type, Plsizei size, void* data, Plint copy);
```

### Parameters

| Name | |
| --- | --- |
| `type` | Specifies the data type to set. Can be PL_VERTEX_BUFFER, PL_INDEX_BUFFER. |
| `size` | The number of vertices if the type is PL_VERTEX_BUFFER, the number of indices if it is PL_INDEX_BUFFER. |
| `data` | Binary data input pointer. |

### Notes

{: .note }
- The following constants are accepted by type:
- PL_VERTEX_BUFFER vertex buffer in xyz order, with 32-bit floating points. data must be an array of floating points (float*)(float[]).
- PL_INDEX_BUFFER index buffer in abc order, with 16-bit integers. data must be an array of 16-bit integers (short[]) (short*).
- The data introduced must not be modified at any point in the lifecycle.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | data must never be null. |
| `—` | size must never be 0. |
| `—` | The errors resulting from this appear when using other related functions. |

## `plGetShapei`

Get a value for a specific parameter

```c
void plGetShapei(Pluint shape, Plenum param, Plint* value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to get the value. Can be PL_SHAPE_TYPE, PL_USER_POINTER, PL_NUM_CHILD_SHAPES. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_SHAPE_TYPE gets the collision shape type.
- PL_USER_POINTER gets the user pointer.
- PL_NUM_CHILD_SHAPES gets the number of children. Must be a compound shape.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the collision shape does not exist or the id is incorrect. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plGetShapefv`

Get a value for a specific parameter

```c
void plGetShapefv(Pluint shape, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `shape` | The id of the collision shape |
| `param` | Specifies the parameter to get the value. Can be PL_TRANSFORM, |
| `values` | Values to get. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_TRANSFORM sets the transformation. values must be a transformation matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_OPERATION` | in rare occasions. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

</div>

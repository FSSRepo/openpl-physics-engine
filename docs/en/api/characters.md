---
title: Kinematic characters
parent: API Reference
grand_parent: Home
nav_order: 8
---

# Kinematic characters

<div class="openpl-api" markdown="1">

## `plGenCharacter`

Generate a character.

```c
Pluint plGenCharacter();
```

### Notes

{: .note }
- This function returns an id and creates a queue of the variables needed before creating an instance of the object.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if there is a pending queue to create a character. |

## `plBindCharacter`

Bind to use a character.

```c
void plBindCharacter(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `chr` | The id of the character to bind. |

### Notes

{: .note }
- Before calling this function to bind a character, make sure it was not bound previously; if so, to unbind, chr must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if a character is already bound previously. |

## `plDeleteCharacter`

Delete a character.

```c
void plDeleteCharacter(Pluint indx);
```

### Parameters

| Name | |
| --- | --- |
| `chr` | The id of the character to delete. |

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the character is bound or is being used. It can also be caused by the character you are trying to delete no longer existing. |

## `plCharacterf`

Set a value for a specific parameter

```c
void plCharacterf(Plenum param, Plfloat val);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_CHAR_STEP_HEIGHT, PL_CHAR_SPHERE, PL_CHAR_ANGULAR_DAMPING, PL_CHAR_LINEAR_DAMPING, PL_CHAR_PRE_STEP, PL_CHAR_PLAYER_STEP, PL_CHAR_JUMP_SPEED, PL_CHAR_FALL_SPEED, PL_CHAR_JUMP, PL_CHAR_MAX_SLOPE, PL_CHAR_MAX_JUMP_HEIGHT, PL_CHAR_MAX_PENETRATION_DEPTH. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CHAR_STEP_HEIGHT sets the step height.
- PL_CHAR_SPHERE sets a sphere.
- PL_CHAR_ANGULAR_DAMPING sets the angular damping
- PL_CHAR_ANGULAR_DAMPING sets the linear damping
- PL_CHAR_PRE_STEP pre step*
- PL_CHAR_PLAYER_STEP player step*
- PL_CHAR_JUMP_SPEED sets the jump speed
- PL_CHAR_FALL_SPEED sets the fall speed
- PL_CHAR_JUMP jump
- PL_CHAR_MAX_SLOPE max slope*
- PL_CHAR_MAX_JUMP_HEIGHT maximum jump height
- PL_CHAR_MAX_PENETRATION_DEPTH penetration depth.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the character does not exist or the id is incorrect. |

## `plCharacter3f`

Set a value for a specific parameter

```c
void plCharacter3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_CHAR_UPVEC, PL_CHAR_CAPSULE_X, PL_CHAR_CAPSULE_Y, PL_CHAR_CAPSULE_Z, PL_CHAR_BOX, PL_CHAR_WALK_DIRECTION, PL_CHAR_JUMP PL_CHAR_APPLY_IMPULSE, PL_CHAR_ANGULAR_VELOCITY, PL_CHAR_LINEAR_VELOCITY. |
| `x y z` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CHAR_UPVEC sets the up vector.
- PL_CHAR_CAPSULE_X sets a capsule oriented on the x axis.
- PL_CHAR_CAPSULE_Y sets a capsule oriented on the y axis.
- PL_CHAR_CAPSULE_Z sets a capsule oriented on the z axis.
- PL_CHAR_BOX sets a box.
- PL_CHAR_WALK_DIRECTION sets the walk direction.
- PL_CHAR_JUMP jump in a direction.
- PL_CHAR_APPLY_IMPULSE applies an impulse.
- PL_CHAR_ANGULAR_VELOCITY sets the angular velocity.
- PL_CHAR_LINEAR_VELOCITY sets the linear velocity.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the character does not exist or the id is incorrect. |

## `plCharacterfv`

```c
void plCharacterfv(Plenum param, Plfloat* values);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Plenum |
| `values` | Plfloat* |

## `plGetCharacterf`

Get a value for a specific parameter

```c
void plGetCharacterf(Pluint indx, Plenum param, Plfloat* value);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Character ID |
| `param` | Specifies the parameter to get the value. Can be PL_CHAR_CAN_JUMP, |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CHAR_CAN_JUMP gets 1 or 0 if the character can jump.

### Errors

| Code | When it happens |
| --- | --- |
| `—` | Errors are not very common |

## `plGetCharacterb`

```c
PLbool plGetCharacterb(Pluint indx, Plenum param);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Pluint |
| `param` | Plenum |

## `plGetCharacterfv`

Get a value for a specific parameter

```c
void plGetCharacterfv(Pluint indx, Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `indx` | Character ID |
| `param` | Specifies the parameter to get the value. Can be PL_TRANSFORM, |
| `values` | Values to get. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_TRANSFORM gets the character transformation. values is a Matrix4x4. lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |

</div>

---
title: Bodies
parent: API Reference
grand_parent: Home
nav_order: 3
---

# Bodies

<div class="openpl-api" markdown="1">

## `plGenBody`

Generate a rigid body.

```c
Pluint plGenBody();
```

### Notes

{: .note }
- This function returns an id and creates a queue of the variables needed before creating an instance of the object.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if there is a pending queue to create a rigid body. |

## `plDeleteBody`

Delete a rigid body.

```c
void plDeleteBody(Pluint body);
```

### Parameters

| Name | |
| --- | --- |
| `body` | The id of the rigid body to delete. |

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body is bound or is being used. It can also be caused by the rigid body you are trying to delete no longer existing. |

## `plBindBody`

Bind to use a rigid body.

```c
void plBindBody(Pluint body);
```

### Parameters

| Name | |
| --- | --- |
| `body` | The id of the rigid body to bind. |

### Notes

{: .note }
- Before calling this function to bind a rigid body, make sure it was not bound previously; if so, to unbind, body must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if a rigid body is already bound previously. |

</div>

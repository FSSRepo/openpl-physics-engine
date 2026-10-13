---
title: Constraints
parent: API Reference
grand_parent: Home
nav_order: 6
---

# Constraints

<div class="openpl-api" markdown="1">

## `plGenConstraint`

Generate a rigid body.

```c
Pluint plGenConstraint(Plenum type);
```

### Parameters

| Name | |
| --- | --- |
| `type` | Specify the type of constraint to create. |

### Notes

{: .note }
- Constraint types:
- This function returns an id and creates a queue of the variables needed before creating an instance of the object.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if there is a pending queue to create a rigid body. |

## `plBindConstraint`

Bind to use a constraint.

```c
void plBindConstraint(Pluint ctr);
```

### Parameters

| Name | |
| --- | --- |
| `ctr` | The id of the constraint to bind. |

### Notes

{: .note }
- Before calling this function to bind a constraint, make sure it was not bound previously; if so, to unbind, shape must be 0.

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_OPERATION` | if a constraint is already bound previously. |

## `plDeleteConstraint`

Delete a constraint.

```c
void plDeleteConstraint(Pluint ctr);
```

### Parameters

| Name | |
| --- | --- |
| `ctr` | The id of the constraint to delete. |

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the constraint is bound or is being used. It can also be caused by the constraint you are trying to delete no longer existing. |

## `plConstrainti`

Set a value for a specific parameter

```c
void plConstrainti(Plenum param, Plint value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_CONSTR_RBODY_A, PL_CONSTR_RBODY_B, PL_CONSTR_USE_REFERENCE_A, PL_HC_MAX_MOTOR_IMPULSE, PL_HC_ENABLE_ANGULAR_MOTOR, PL_G6DS_ENABLE_SPRING, PL_G6DS_DISABLE_SPRING, PL_G6DS_EQUILI_POINT_SPRING, PL_G6DS2_ENABLE_SPRING, PL_G6DS2_DISABLE_SPRING, PL_G6DS2_EQUILI_POINT_SPRING, PL_G6DS2_ENABLE_MOTOR, PL_G6DS2_SET_SERVO, PL_SLDR_POWERED_LINEAR_MOTOR, PL_SLDR_POWERED_ANGULAR_MOTOR. |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CONSTR_RBODY_A binds a rigid body.
- PL_CONSTR_RBODY_B binds a rigid body.
- PL_CONSTR_USE_REFERENCE_A use reference
- PL_HC_MAX_MOTOR_IMPULSE setAngularOnly (reference function) see the class documentation in Bullet Physics
- PL_HC_ENABLE_ANGULAR_MOTOR enableMotor (reference function) see the class documentation in Bullet Physics
- PL_G6DS_ENABLE_SPRING enableSpring = true (reference function) see the class documentation in Bullet Physics
- PL_G6DS_DISABLE_SPRING enableSpring = false (reference function) see the class documentation in Bullet Physics
- PL_G6DS_EQUILI_POINT_SPRING setEquilibriumPoint (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_ENABLE_SPRING enableSpring = true (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_DISABLE_SPRING enableSpring = false (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_EQUILI_POINT_SPRING setEquilibriumPoint (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_ENABLE_MOTOR enableMotor (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_SET_SERVO setServo (reference function) see the class documentation in Bullet Physics
- PL_SLDR_POWERED_LINEAR_MOTOR setPoweredLinMotor (reference function) see the class documentation in Bullet Physics
- PL_SLDR_POWERED_ANGULAR_MOTOR setPoweredAngMotor (reference function) see the class documentation in Bullet Physics

### Errors

| Code | When it happens |
| --- | --- |
| `—` | No errors occur. |

## `plConstraintf`

Set a value for a specific parameter

```c
void plConstraintf(Plenum param, Plfloat value);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_HC_MAX_MOTOR_IMPULSE, PL_HC_TARGET_VELOCITY, PL_SLDR_SOFTNESS_DIR_LINEAR, PL_SLDR_SOFTNESS_DIR_ANGULAR, PL_SLDR_SOFTNESS_LIMIT_LIN, PL_SLDR_SOFTNESS_ORTHO_LIN, PL_SLDR_SOFTNESS_ORTHO_ANG, PL_SLDR_DAMPING_DIR_LINEAR, PL_SLDR_DAMPING_DIR_ANGULAR, PL_SLDR_DAMPING_LIMIT_LIN, PL_SLDR_DAMPING_ORTHO_LIN, PL_SLDR_DAMPING_ORTHO_ANG, PL_SLDR_RESTITUTION_DIR_LINEAR, PL_SLDR_RESTITUTION_DIR_ANGULAR, PL_SLDR_RESTITUTION_LIMIT_LIN, PL_SLDR_RESTITUTION_ORTHO_LIN, PL_SLDR_RESTITUTION_ORTHO_ANG, PL_SLDR_MAX_ANG_MOTOR_FORCE, PL_SLDR_MAX_LIN_MOTOR_FORCE, PL_SLDR_TARGET_LIN_MOTOR_VEL, PL_SLDR_TARGET_ANG_MOTOR_VEL, PL_HC2_LOWER_LIMIT, PL_HC2_UPPER_LIMIT, PL_P2P_UPDATE_RHS, |
| `value` | The assigned value. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_HC_MAX_MOTOR_IMPULSE setMaxMotorImpulse (reference function) see the class documentation in Bullet Physics
- PL_HC_TARGET_VELOCITY setMotorTargetVelocity (reference function) see the class documentation in Bullet Physics
- PL_SLDR_SOFTNESS_DIR_LINEAR setSoftnessDirLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_SOFTNESS_DIR_ANGULAR setSoftnessDirAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_SOFTNESS_LIMIT_LIN setSoftnessLimLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_SOFTNESS_ORTHO_LIN setSoftnessOrthoLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_SOFTNESS_ORTHO_ANG setSoftnessOrthoAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_DAMPING_DIR_LINEAR setDampingDirLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_DAMPING_DIR_ANGULAR setDampingDirAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_DAMPING_LIMIT_LIN setDampingLimLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_DAMPING_ORTHO_LIN setDampingOrthoLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_DAMPING_ORTHO_ANG setDampingOrthoAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_RESTITUTION_DIR_LINEAR setRestitutionDirLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_RESTITUTION_DIR_ANGULAR setRestitutionDirAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_RESTITUTION_LIMIT_LIN setRestitutionLimLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_RESTITUTION_ORTHO_LIN setRestitutionOrthoLin (reference function) see the class documentation in Bullet Physics
- PL_SLDR_RESTITUTION_ORTHO_ANG setRestitutionOrthoAng (reference function) see the class documentation in Bullet Physics
- PL_SLDR_MAX_ANG_MOTOR_FORCE setMaxAngMotorForce (reference function) see the class documentation in Bullet Physics
- PL_SLDR_MAX_LIN_MOTOR_FORCE setMaxLinMotorForce (reference function) see the class documentation in Bullet Physics
- PL_SLDR_TARGET_LIN_MOTOR_VEL setTargetLinMotorVelocity (reference function) see the class documentation in Bullet Physics
- PL_SLDR_TARGET_ANG_MOTOR_VEL setTargetAngMotorVelocity (reference function) see the class documentation in Bullet Physics
- PL_HC2_LOWER_LIMIT setLowerLimit (reference function) see the class documentation in Bullet Physics
- PL_HC2_UPPER_LIMIT setUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_P2P_UPDATE_RHS updateRHS (reference function) see the class documentation in Bullet Physics

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the constraint does not exist or the id is incorrect. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plConstraint3f`

Set a value for a specific parameter

```c
void plConstraint3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_CONSTR_PIVOT_A, PL_CONSTR_PIVOT_B, PL_CONSTR_IN_AXIS_A, PL_CONSTR_IN_AXIS_B, PL_CTWIST_LIMITS, PL_HC_LIMITS, PL_HC2_LIMITS, PL_G6D_LOWER_LIN_LIMIT, PL_G6D_UPPER_LIN_LIMIT, PL_G6D_LOWER_ANG_LIMIT, PL_G6D_UPPER_ANG_LIMIT, PL_G6D_LIMIT_LIN_X, PL_G6D_LIMIT_LIN_Y, PL_G6D_LIMIT_LIN_Z, PL_G6D_LIMIT_ANG_X, PL_G6D_LIMIT_ANG_Y, PL_G6D_LIMIT_ANG_Z, PL_G6DS_STIFFNESS_SPRING, PL_G6DS_DAMPING_SPRING, PL_G6DS_EQUILI_POINT_SPRING, PL_G6DS2_LOWER_LIN_LIMIT, PL_G6DS2_UPPER_LIN_LIMIT, PL_G6DS2_LOWER_ANG_LIMIT, PL_G6DS2_UPPER_ANG_LIMIT, PL_G6DS2_STIFFNESS_SPRING, PL_G6DS2_DAMPING_SPRING, PL_G6DS2_EQUILI_POINT_SPRING, PL_G6DS2_TARGET_VELOCITY, PL_G6DS2_SERVO_TARGET, PL_G6DS2_MAX_MOTOR_FORCE, PL_G6DS2_BOUNCE, PL_P2P_PIVOT_A, PL_P2P_PIVOT_B. |
| `x y z` | The assigned values. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CONSTR_PIVOT_A set pivot A
- PL_CONSTR_PIVOT_B set pivot B
- PL_CONSTR_IN_AXIS_A set axis A
- PL_CONSTR_IN_AXIS_B set axis B
- PL_CTWIST_LIMITS setLimit (reference function) see the class documentation in Bullet Physics
- PL_HC_LIMITS setLimit (reference function) see the class documentation in Bullet Physics
- PL_HC2_LIMITS setUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LOWER_LIN_LIMIT setLinearLowerLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_UPPER_LIN_LIMIT setLinearUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LOWER_ANG_LIMIT setAngularLowerLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_UPPER_ANG_LIMIT setAngularUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_LIN_X setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_LIN_Y setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_LIN_Z setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_ANG_X setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_ANG_Y setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6D_LIMIT_ANG_Z setLimit (reference function) see the class documentation in Bullet Physics
- PL_G6DS_STIFFNESS_SPRING setStiffness (reference function) see the class documentation in Bullet Physics
- PL_G6DS_DAMPING_SPRING setDamping (reference function) see the class documentation in Bullet Physics
- PL_G6DS_EQUILI_POINT_SPRING setEquilibriumPoint (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_LOWER_LIN_LIMIT setLinearLowerLimit (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_UPPER_LIN_LIMIT setLinearUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_LOWER_ANG_LIMIT setAngularLowerLimit (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_UPPER_ANG_LIMIT setAngularUpperLimit (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_STIFFNESS_SPRING setStiffness (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_DAMPING_SPRING setDamping (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_EQUILI_POINT_SPRING setEquilibriumPoint (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_TARGET_VELOCITY setTargetVelocity (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_SERVO_TARGET setServoTarget (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_MAX_MOTOR_FORCE setMaxMotorForce (reference function) see the class documentation in Bullet Physics
- PL_G6DS2_BOUNCE setBounce (reference function) see the class documentation in Bullet Physics
- PL_P2P_PIVOT_A setPivotA (reference function) see the class documentation in Bullet Physics
- PL_P2P_PIVOT_B setPivotB (reference function) see the class documentation in Bullet Physics

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the constraint does not exist or the id is incorrect. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

## `plConstraintfv`

Set a value for a specific parameter

```c
void plConstraintfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parameters

| Name | |
| --- | --- |
| `param` | Specifies the parameter to set the value for. Can be PL_CONSTR_FRAME_A, PL_CONSTR_FRAME_B. |
| `values` | The assigned values. |
| `lenght` | The length of the values array. |

### Notes

{: .note }
- The following constants are accepted by param:
- PL_CONSTR_FRAME_A set transformation. matrix4x4 lenght must be 16
- PL_CONSTR_FRAME_B set transformation. matrix4x4 lenght must be 16

### Errors

| Code | When it happens |
| --- | --- |
| `PL_INVALID_NAME` | if the rigid body does not exist or the id is incorrect. |
| `PL_INVALID_ENUM` | if param is an invalid or disallowed constant. |

</div>

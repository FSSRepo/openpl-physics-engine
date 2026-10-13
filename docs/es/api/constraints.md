---
title: Restricciones
parent: Referencia API
grand_parent: Inicio
nav_order: 6
---

# Restricciones

<div class="openpl-api" markdown="1">

## `plGenConstraint`

Generar un cuerpo rigido.

```c
Pluint plGenConstraint(Plenum type);
```

### Parámetros

| Nombre | |
| --- | --- |
| `type` | Especificar el tipo de restriccion a crear. |

### Notas

{: .note }
- Tipos de restricciones:
- Esta funcion retorna una id y crea una cola de variables necesarias antes de crear una instancia del objeto.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si hay una cola pendiente para crear un cuerpo rigido. |

## `plBindConstraint`

Enlazar para usar una restriccion.

```c
void plBindConstraint(Pluint ctr);
```

### Parámetros

| Nombre | |
| --- | --- |
| `ctr` | La id de la restriccion a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar una restriccion asegurate de que no haya sido enlazado previamente, si es asi para desenlazar shape debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si ya hay una restriccion enlazada previamente. |

## `plDeleteConstraint`

Eliminar una restriccion.

```c
void plDeleteConstraint(Pluint ctr);
```

### Parámetros

| Nombre | |
| --- | --- |
| `ctr` | La id de la restriccion a eliminar. |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si la restriccion esta enlazado o esta siendo usado. Tambien puede ser causado por que la restriccion que tratas de eliminar ya no existe. |

## `plConstrainti`

Poner un valor a un parametro especifico

```c
void plConstrainti(Plenum param, Plint value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CONSTR_RBODY_A, PL_CONSTR_RBODY_B, PL_CONSTR_USE_REFERENCE_A, PL_HC_MAX_MOTOR_IMPULSE, PL_HC_ENABLE_ANGULAR_MOTOR, PL_G6DS_ENABLE_SPRING, PL_G6DS_DISABLE_SPRING, PL_G6DS_EQUILI_POINT_SPRING, PL_G6DS2_ENABLE_SPRING, PL_G6DS2_DISABLE_SPRING, PL_G6DS2_EQUILI_POINT_SPRING, PL_G6DS2_ENABLE_MOTOR, PL_G6DS2_SET_SERVO, PL_SLDR_POWERED_LINEAR_MOTOR, PL_SLDR_POWERED_ANGULAR_MOTOR. |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CONSTR_RBODY_A enlazar un cuerpo rigido.
- PL_CONSTR_RBODY_B enlazar un cuerpo rigido.
- PL_CONSTR_USE_REFERENCE_A usar referencia
- PL_HC_MAX_MOTOR_IMPULSE setAngularOnly (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC_ENABLE_ANGULAR_MOTOR enableMotor (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_ENABLE_SPRING enableSpring = true (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_DISABLE_SPRING enableSpring = false (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_EQUILI_POINT_SPRING setEquilibriumPoint (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_ENABLE_SPRING enableSpring = true (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_DISABLE_SPRING enableSpring = false (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_EQUILI_POINT_SPRING setEquilibriumPoint (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_ENABLE_MOTOR enableMotor (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_SET_SERVO setServo (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_POWERED_LINEAR_MOTOR setPoweredLinMotor (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_POWERED_ANGULAR_MOTOR setPoweredAngMotor (funcion de referencia) ver documentacion de la clase en BulletPhyics

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `—` | No se producen errores. |

## `plConstraintf`

Poner un valor a un parametro especifico

```c
void plConstraintf(Plenum param, Plfloat value);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_HC_MAX_MOTOR_IMPULSE, PL_HC_TARGET_VELOCITY, PL_SLDR_SOFTNESS_DIR_LINEAR, PL_SLDR_SOFTNESS_DIR_ANGULAR, PL_SLDR_SOFTNESS_LIMIT_LIN, PL_SLDR_SOFTNESS_ORTHO_LIN, PL_SLDR_SOFTNESS_ORTHO_ANG, PL_SLDR_DAMPING_DIR_LINEAR, PL_SLDR_DAMPING_DIR_ANGULAR, PL_SLDR_DAMPING_LIMIT_LIN, PL_SLDR_DAMPING_ORTHO_LIN, PL_SLDR_DAMPING_ORTHO_ANG, PL_SLDR_RESTITUTION_DIR_LINEAR, PL_SLDR_RESTITUTION_DIR_ANGULAR, PL_SLDR_RESTITUTION_LIMIT_LIN, PL_SLDR_RESTITUTION_ORTHO_LIN, PL_SLDR_RESTITUTION_ORTHO_ANG, PL_SLDR_MAX_ANG_MOTOR_FORCE, PL_SLDR_MAX_LIN_MOTOR_FORCE, PL_SLDR_TARGET_LIN_MOTOR_VEL, PL_SLDR_TARGET_ANG_MOTOR_VEL, PL_HC2_LOWER_LIMIT, PL_HC2_UPPER_LIMIT, PL_P2P_UPDATE_RHS, |
| `value` | El valor asignado. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_HC_MAX_MOTOR_IMPULSE setMaxMotorImpulse (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC_TARGET_VELOCITY setMotorTargetVelocity (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_SOFTNESS_DIR_LINEAR setSoftnessDirLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_SOFTNESS_DIR_ANGULAR setSoftnessDirAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_SOFTNESS_LIMIT_LIN setSoftnessLimLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_SOFTNESS_ORTHO_LIN setSoftnessOrthoLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_SOFTNESS_ORTHO_ANG setSoftnessOrthoAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_DAMPING_DIR_LINEAR setDampingDirLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_DAMPING_DIR_ANGULAR setDampingDirAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_DAMPING_LIMIT_LIN setDampingLimLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_DAMPING_ORTHO_LIN setDampingOrthoLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_DAMPING_ORTHO_ANG setDampingOrthoAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_RESTITUTION_DIR_LINEAR setRestitutionDirLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_RESTITUTION_DIR_ANGULAR setRestitutionDirAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_RESTITUTION_LIMIT_LIN setRestitutionLimLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_RESTITUTION_ORTHO_LIN setRestitutionOrthoLin (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_RESTITUTION_ORTHO_ANG setRestitutionOrthoAng (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_MAX_ANG_MOTOR_FORCE setMaxAngMotorForce (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_MAX_LIN_MOTOR_FORCE setMaxLinMotorForce (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_TARGET_LIN_MOTOR_VEL setTargetLinMotorVelocity (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_SLDR_TARGET_ANG_MOTOR_VEL setTargetAngMotorVelocity (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC2_LOWER_LIMIT setLowerLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC2_UPPER_LIMIT setUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_P2P_UPDATE_RHS updateRHS (funcion de referencia) ver documentacion de la clase en BulletPhyics

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si la restriccion no existe o el id es incorrecto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plConstraint3f`

Poner un valor a un parametro especifico

```c
void plConstraint3f(Plenum param, Plfloat x, Plfloat y, Plfloat z);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CONSTR_PIVOT_A, PL_CONSTR_PIVOT_B, PL_CONSTR_IN_AXIS_A, PL_CONSTR_IN_AXIS_B, PL_CTWIST_LIMITS, PL_HC_LIMITS, PL_HC2_LIMITS, PL_G6D_LOWER_LIN_LIMIT, PL_G6D_UPPER_LIN_LIMIT, PL_G6D_LOWER_ANG_LIMIT, PL_G6D_UPPER_ANG_LIMIT, PL_G6D_LIMIT_LIN_X, PL_G6D_LIMIT_LIN_Y, PL_G6D_LIMIT_LIN_Z, PL_G6D_LIMIT_ANG_X, PL_G6D_LIMIT_ANG_Y, PL_G6D_LIMIT_ANG_Z, PL_G6DS_STIFFNESS_SPRING, PL_G6DS_DAMPING_SPRING, PL_G6DS_EQUILI_POINT_SPRING, PL_G6DS2_LOWER_LIN_LIMIT, PL_G6DS2_UPPER_LIN_LIMIT, PL_G6DS2_LOWER_ANG_LIMIT, PL_G6DS2_UPPER_ANG_LIMIT, PL_G6DS2_STIFFNESS_SPRING, PL_G6DS2_DAMPING_SPRING, PL_G6DS2_EQUILI_POINT_SPRING, PL_G6DS2_TARGET_VELOCITY, PL_G6DS2_SERVO_TARGET, PL_G6DS2_MAX_MOTOR_FORCE, PL_G6DS2_BOUNCE, PL_P2P_PIVOT_A, PL_P2P_PIVOT_B. |
| `x y z` | Los valores asignados. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CONSTR_PIVOT_A poner pivote A
- PL_CONSTR_PIVOT_B poner pivote B
- PL_CONSTR_IN_AXIS_A poner axis A
- PL_CONSTR_IN_AXIS_B poner axis B
- PL_CTWIST_LIMITS setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC_LIMITS setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_HC2_LIMITS setUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LOWER_LIN_LIMIT setLinearLowerLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_UPPER_LIN_LIMIT setLinearUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LOWER_ANG_LIMIT setAngularLowerLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_UPPER_ANG_LIMIT setAngularUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_LIN_X setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_LIN_Y setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_LIN_Z setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_ANG_X setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_ANG_Y setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6D_LIMIT_ANG_Z setLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_STIFFNESS_SPRING setStiffness (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_DAMPING_SPRING setDamping (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS_EQUILI_POINT_SPRING setEquilibriumPoint (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_LOWER_LIN_LIMIT setLinearLowerLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_UPPER_LIN_LIMIT setLinearUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_LOWER_ANG_LIMIT setAngularLowerLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_UPPER_ANG_LIMIT setAngularUpperLimit (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_STIFFNESS_SPRING setStiffness (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_DAMPING_SPRING setDamping (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_EQUILI_POINT_SPRING setEquilibriumPoint (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_TARGET_VELOCITY setTargetVelocity (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_SERVO_TARGET setServoTarget (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_MAX_MOTOR_FORCE setMaxMotorForce (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_G6DS2_BOUNCE setBounce (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_P2P_PIVOT_A setPivotA (funcion de referencia) ver documentacion de la clase en BulletPhyics
- PL_P2P_PIVOT_B setPivotB (funcion de referencia) ver documentacion de la clase en BulletPhyics

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si la restriccion no existe o el id es incorrecto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

## `plConstraintfv`

Poner un valor a un parametro especifico

```c
void plConstraintfv(Plenum param, Plfloat* values, Plsizei lenght);
```

### Parámetros

| Nombre | |
| --- | --- |
| `param` | Especifica el parametro para introducir el valor. Puede ser PL_CONSTR_FRAME_A, PL_CONSTR_FRAME_B. |
| `values` | Los valores asignados. |
| `lenght` | La longitud del arreglo de valores. |

### Notas

{: .note }
- Las siguientes constantes son aceptadas por param:
- PL_CONSTR_FRAME_A poner transformacion. matrix4x4lenght debe ser 16
- PL_CONSTR_FRAME_B poner transformacion. matrix4x4 lenght debe ser 16

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido no existe o el id es incorrecto. |
| `PL_INVALID_ENUM` | si param es una constante invalida o no permitida. |

</div>

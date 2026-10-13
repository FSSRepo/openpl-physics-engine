---
title: Cuerpos
parent: Referencia API
grand_parent: Inicio
nav_order: 3
---

# Cuerpos

<div class="openpl-api" markdown="1">

## `plGenBody`

Generar un cuerpo rigido.

```c
Pluint plGenBody();
```

### Notas

{: .note }
- Esta funcion retorna una id y crea una cola de variables necesarias antes de crear una instancia del objeto.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si hay una cola pendiente para crear un cuerpo rigido. |

## `plDeleteBody`

Eliminar un cuerpo rigido.

```c
void plDeleteBody(Pluint body);
```

### Parámetros

| Nombre | |
| --- | --- |
| `body` | La id del cuerpo rigido a eliminar. |

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_NAME` | si el cuerpo rigido esta enlazado o esta siendo usado. Tambien puede ser causado por que el cuerpo rigido que tratas de eliminar ya no existe. |

## `plBindBody`

Enlazar para usar un cuerpo rigido.

```c
void plBindBody(Pluint body);
```

### Parámetros

| Nombre | |
| --- | --- |
| `body` | La id del cuerpo rigido a enlazar. |

### Notas

{: .note }
- Antes de llamar esta funcion para enlazar un cuerpo rigido asegurate de que no haya sido enlazado previamente, si es asi para desenlazar body debe ser 0.

### Errores

| Código | Cuándo ocurre |
| --- | --- |
| `PL_INVALID_OPERATION` | si ya hay un cuerpo rigido enlazado previamente. |

</div>

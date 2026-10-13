---
layout: default
title: Example
parent: Guide
nav_order: 4
---

# Example

A minimal OpenPL skeleton. Notice the **queued** pattern of the API: first a resource is
**generated**, then its parameters are supplied, and finally it is **created** by stating
the concrete type.

```cpp
#include <openpl/bullet.h>
#include <iostream>

using namespace std;

/*
    Fast Smart System
    cross-platform open source project
*/

int main() {
    // 1. Create the global context.
    if (!plCreateContext()) {
        cout << "Can't create the context" << endl;
        return 1;
    }

    // 2. Generate a queued resource (a shape).
    int shape = plGenShape();

    // 3. Supply its parameters.
    plShapef(PL_RADIUS, 3.0f);

    // 4. Create the resource by stating the concrete type.
    plCreate(PL_SPHERE_SHAPE);

    // 5. Select the generated resource.
    plBindShape(shape);

    // 6. Release the context.
    plDestroyContext();
    return 0;
}
```

{: .note }
`plGenShape()` returns an **id** that is referenced later with `plBindShape()`. See the
[API reference](../api/index.html) for the rest of the functions.

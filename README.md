# isect2d

![00](img/capture.png)

Collision detection algorithms, using basic tile grid for broad-phase and separating axes theorem
(SAT) for narrow-phase.

Example
-------

```cpp
// Broad-phase
std::vector<isect2d::OBB> obbs;
std::vector<isect2d::AABB> aabbs;

for (auto& obb : obbs) {
    auto aabb = obb.getExtent();
    aabbs.push_back(aabb);
}

auto pairs = intersect(
    aabbs, 
    {4, 4},     // split resolution  
    {800, 600}, // screen resolution
);

// Narrow-phase
for (auto& pair : pairs) {
    if (intersect(obbs[pair.first], obbs[pair.second])) {
        // Both oriented bounding boxes collide
    }
}
```

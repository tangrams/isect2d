#include "../src/isect2d.h"

#include <iostream>

int main() {
    isect2d::OBB obb1(0.8, 0.5, M_PI_4, 1.0, 1.0);
    isect2d::OBB obb2(0.5, 0.5, 0.0, 1.0, 1.0);

    std::cout << intersect(obb1, obb2) << std::endl;

    return 0;
}


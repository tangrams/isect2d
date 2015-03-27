#define SEC2D_IMPLEMENTATION
#include "src/sec2d.h"

#include <iostream>

int main() {
    sec2d::OBB obb1(1.5, 1.5, M_PI_4, 1.0, 1.0);
    sec2d::OBB obb2(0.5, 0.5, 0.0, 1.0, 1.0);
    
    std::cout << intersect(obb1, obb2) << std::endl;

    return 0;
}


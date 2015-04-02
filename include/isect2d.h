#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <iostream>

#include "aabb.h"
#include "vec2.h"
#include "bvh.h"

static std::set<std::pair<int, int>> intersect(const isect2d::AABB* _aabbs1, const size_t _size1,
                                               const isect2d::AABB* _aabbs2, const size_t _size2) {
    std::set<std::pair<int, int>> pairs;

    // bruteforce
    for (int i = 0; i < _size1; ++i) {
        for (int j = i + 1; j < _size2; ++j) {
            if (_aabbs1[i].intersect(_aabbs2[j])) {
                pairs.insert({ i, j });
            }
        }
    }

    return pairs;
}


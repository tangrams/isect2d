#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <vector>
#include <iostream>

#include "aabb.h"
#include "vec2.h"

#define BRUTEFORCE

static void intersect(const std::vector<isect2d::AABB>& _aabbs, std::set<std::pair<void*, void*>>& _pairs) {

    if (_aabbs.size() == 0) {
        return;
    }

    for (int i = 0; i < _aabbs.size(); ++i) {
        for (int j = i + 1; j < _aabbs.size(); ++j) {
            if (_aabbs[i].intersect(_aabbs[j])) {
                _pairs.insert({ _aabbs[i].m_userData, _aabbs[j].m_userData });
            }
        }
    }


    return;
}


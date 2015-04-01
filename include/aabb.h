#pragma once

#include "vec2.h"
#include <cmath>
#include <algorithm>

namespace isect2d {

struct AABB {
    AABB(float _minx, float _miny, float _maxx, float _maxy) : m_min(_minx, _miny), m_max(_maxx, _maxy) {

    }

    bool intersect(const AABB& _other) const {
    	return _other.m_max.x >= m_min.x &&
    		   _other.m_max.y >= m_min.y &&
    		   _other.m_min.x <= m_max.x &&
    		   _other.m_min.y <= m_max.y;
    }

    const Vec2& getMin() const {
        return m_min;
    }

    const Vec2& getMax() const {
        return m_max;
    }

    void* m_userData;

private:
    Vec2 m_min;
    Vec2 m_max;

};

}

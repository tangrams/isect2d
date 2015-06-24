#pragma once

#include "vec2.h"
#include <cmath>
#include <algorithm>

namespace isect2d {

enum Dimension {
    X, Y
};

struct AABB {
    AABB() : AABB(INT_MAX, INT_MAX, -INT_MAX, -INT_MAX) {}

    AABB(float _minx, float _miny, float _maxx, float _maxy)
        : m_min(_minx, _miny), m_max(_maxx, _maxy) {
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

    Vec2 getCentroid() const {
        return (m_min + m_max) * 0.5;
    }

    Dimension maxExtent() {
        Vec2 diagonal = m_max - m_min;
        if(diagonal.x > diagonal.y) {
            return X;
        }
        return Y;
    }

    void* m_userData;

    Vec2 m_min;
    Vec2 m_max;

};

static inline AABB unionAABB(const AABB& _aabb1, const AABB& _aabb2) {
    AABB aabb;

    aabb.m_min.x = std::min(_aabb1.m_min.x, _aabb2.m_min.x);
    aabb.m_min.y = std::min(_aabb1.m_min.y, _aabb2.m_min.y);
    aabb.m_max.x = std::max(_aabb1.m_max.x, _aabb2.m_max.x);
    aabb.m_max.y = std::max(_aabb1.m_max.y, _aabb2.m_max.y);

    return aabb;
}

inline bool operator==(const AABB& lh, const AABB& rh) {
    return lh.m_min == rh.m_min && lh.m_max == rh.m_max;
}

inline bool operator!=(const AABB& lh, const AABB& rh) {
    return !(lh == rh);
}

}

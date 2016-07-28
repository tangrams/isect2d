#pragma once

#include <cmath>
#include <climits>
#include <algorithm>
#include <climits>

namespace isect2d {

enum Dimension {
    X, Y
};

template<typename V>
struct AABB {
    AABB() : AABB(INT_MAX, INT_MAX, -INT_MAX, -INT_MAX) {}

    AABB(float _minx, float _miny, float _maxx, float _maxy)
        : min(_minx, _miny), max(_maxx, _maxy) {
    }

    inline bool intersect(const AABB& _other) const {
        return _other.max.x >= min.x &&
                   _other.max.y >= min.y &&
                   _other.min.x <= max.x &&
                   _other.min.y <= max.y;
    }

    const V& getMin() const {
        return min;
    }

    const V& getMax() const {
        return max;
    }

    V getCentroid() const {
        return (min + max) * 0.5;
    }

    Dimension maxExtent() {
        V diagonal = max - min;
        if(diagonal.x > diagonal.y) {
            return X;
        }
        return Y;
    }

    void include(float _x, float _y) {
        min.x = std::min(min.x, _x);
        min.y = std::min(min.y, _y);
        max.x = std::max(max.x, _x);
        max.y = std::max(max.y, _y);
    }

    void* m_userData = nullptr;

    V min;
    V max;

};

template<typename V>
static inline AABB<V> unionAABB(const AABB<V>& _aabb1, const AABB<V>& _aabb2) {
    AABB<V> aabb;

    aabb.min.x = std::min(_aabb1.min.x, _aabb2.min.x);
    aabb.min.y = std::min(_aabb1.min.y, _aabb2.min.y);
    aabb.max.x = std::max(_aabb1.max.x, _aabb2.max.x);
    aabb.max.y = std::max(_aabb1.max.y, _aabb2.max.y);

    return aabb;
}

template<typename V>
inline bool operator==(const AABB<V>& lh, const AABB<V>& rh) {
    return lh.min == rh.min && lh.max == rh.max;
}

template<typename V>
inline bool operator!=(const AABB<V>& lh, const AABB<V>& rh) {
    return !(lh == rh);
}

}

#pragma once

#include <algorithm>
#include <array>
#include "vec.h"

namespace isect2d {

template<typename V>
struct OBB {

    OBB() {}

    OBB(float _cx, float _cy, float _a, float _w, float _h) :
        m_width(_w), m_height(_h), m_angle(_a),
        m_centroid(V(_cx, _cy)) {

        update();
    }

    void move(const float _px, const float _py) {
        m_centroid = V(_px, _py);

        update();
    }

    void rotate(float _angle) {
        m_angle = _angle;

        update();
    }

    const std::array<V, 4>& getQuad() const {
        return m_quad;
    }

    const std::array<V, 2>& getAxes() const{
        return m_axes;
    }

    const V& getCentroid() const {
        return m_centroid;
    }

    float getAngle() const {
        return m_angle;
    }

    float getWidth() const {
        return m_width;
    }

    float getHeight() const {
        return m_height;
    }

    float radius() const {
        V extent(m_width, m_height);

        return extent.length();
    }

    AABB<V> getExtent() const {
        float inf = std::numeric_limits<double>::infinity();
        float aabb[4] = { inf, inf, -inf, -inf };

        for (int i = 0; i < 4; ++i) {
            aabb[0] = std::min<float>(m_quad[i].x, aabb[0]);
            aabb[1] = std::min<float>(m_quad[i].y, aabb[1]);
            aabb[2] = std::max<float>(m_quad[i].x, aabb[2]);
            aabb[3] = std::max<float>(m_quad[i].y, aabb[3]);
        }

        return { aabb[0], aabb[1], aabb[2], aabb[3] };
    }

private:

    void perpAxes() {
        m_axes[0] = normalize(m_quad[2] - m_quad[3]);
        m_axes[1] = normalize(m_quad[2] - m_quad[1]);
    }

    void update() {
        V x;
        V y;

        if (m_angle == 0) {
            x = { m_width / 2, 0 };
            y = { 0, m_height / 2 };
        } else {
            float cosa = cos(m_angle);
            float sina = sin(m_angle);

            x = { cosa , sina };
            y = { -sina, cosa };

            x = x * (m_width / 2);
            y = y * (m_height / 2);
        }

        m_quad[0] = m_centroid - x - y; // lower-left
        m_quad[1] = m_centroid + x - y; // lower-right
        m_quad[2] = m_centroid + x + y; // uper-right
        m_quad[3] = m_centroid - x + y; // uper-left

        perpAxes();
    }

    float m_width;
    float m_height;
    float m_angle;

    std::array<V, 2> m_axes;
    V m_centroid;
    std::array<V, 4> m_quad;

};

template<typename V>
inline bool operator==(const OBB<V>& lh, const OBB<V>& rh) {
    return lh.getCentroid() == rh.getCentroid() && lh.getAngle() == rh.getAngle();
}

template<typename V>
inline bool operator!=(const OBB<V>& lh, const OBB<V>& rh) {
    return !(lh == rh);
}

template<typename V>
static V projectToAxis(const OBB<V>& _obb, const V& axis) {
    double inf = std::numeric_limits<double>::infinity();
    double min = inf;
    double max = -inf;

    const std::array<V, 4>& p = _obb.getQuad();

    for (int i = 0; i < 4; ++i) {
        double d = dot(p[i], axis);

        min = std::min(min, d);
        max = std::max(max, d);
    }

    return V(min, max);
}

template<typename V>
inline static bool axisCollide(const OBB<V>& _a, const OBB<V>& _b, const std::array<V, 2> axes) {
    for (int i = 0; i < 2; ++i) {
        V aproj = projectToAxis(_a, axes[i]);
        V bproj = projectToAxis(_b, axes[i]);

        if (bproj.x > aproj.y || bproj.y < aproj.x) {
            return false;
        }
    }

    return true;
}

template<typename V>
inline static bool intersect(const OBB<V>& _a, const OBB<V>& _b) {
    return axisCollide(_a, _b, _a.getAxes()) && axisCollide(_a, _b, _b.getAxes());
}

}

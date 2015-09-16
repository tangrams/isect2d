#pragma once

namespace isect2d {

template<>
inline glm::vec2 project(const glm::vec2& _p, const glm::vec2& _axis) {
    float l = glm::length(_axis);
    return _axis * glm::dot(_p, _axis) * (1.0f / (l * l));
}

template<>
inline glm::vec2 normalize(const glm::vec2& _v) {
    return glm::normalize(_v);
}

}

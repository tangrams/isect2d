#pragma once

#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include <cmath>

namespace isect2d {

template<>
inline glm::vec2 normalize(const glm::vec2& _v) {
    return glm::normalize(_v);
}

inline float distance2(const glm::vec2& _v0, const glm::vec2& _v1) {
    return glm::distance2(_v0, _v1);
}

}

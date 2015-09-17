#pragma once

#include "glm/glm.hpp"

namespace isect2d {

template<>
inline glm::vec2 normalize(const glm::vec2& _v) {
    return glm::normalize(_v);
}

}

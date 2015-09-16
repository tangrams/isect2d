#pragma once

namespace isect2d {

template<typename V>
V project(const V& _p, const V& _axis);

// TODO there is probably a better way do define the return type
template<typename V>
decltype(V::x) dot(const V& _v, const V& _b);

template<typename V>
V normalize(const V& _v);

}

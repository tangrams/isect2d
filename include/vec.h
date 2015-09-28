#pragma once

namespace isect2d {

// TODO there is probably a better way do define the return type
template<typename V>
decltype(V::x) dot(const V& _v, const V& _b);

template<typename V>
V normalize(const V& _v);

}

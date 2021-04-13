#pragma once
#include "../math/vec.h"

namespace tracer {
auto intersect_triangle(
    vec3<float> orig,
    vec3<float> dir,
    vec3<float> vert0,
    vec3<float> vert1,
    vec3<float> vert2,
    float& t,
    float& u,
    float& v) -> bool;
} // namespace tracer

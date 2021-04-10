#pragma once

#include "../math/vec.h"

namespace tracer {

struct ray {
    vec3<float> origin;
    vec3<float> dir;

    ray(vec3<float> origin, vec3<float> dir): origin(origin), dir(dir) {}
};

struct camera {
  public:
    camera(vec3<float> lookfrom, vec3<float> lookat, vec3<float> vup, float vfov, float aspect) {
        // vfov is top to bottom in degrees
        float theta = vfov * M_PI / 180;
        float half_height = tan(theta / 2);
        float half_width = aspect * half_height;
        origin = lookfrom;
        w = normalize(lookfrom - lookat);
        u = normalize(cross(vup, w));
        v = cross(w, u);
        lower_left_corner = origin - u * half_width - v * half_height - w;
        horizontal = u * 2.f * half_width;
        vertical = v * 2.f * half_height;
    }

    ray get_ray(float s, float t) {
        return ray(origin, normalize(lower_left_corner + horizontal * s + vertical * t - origin));
    }

    vec3<float> origin;
    vec3<float> lower_left_corner;
    vec3<float> horizontal;
    vec3<float> vertical;
    vec3<float> u, v, w;
};

} // namespace tracer

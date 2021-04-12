#pragma once

#include "../math/vec.h"

#include <algorithm>
#include <vector>

namespace tracer {
struct Material {
    tracer::vec3<float> ka;
    tracer::vec3<float> kd;
    tracer::vec3<float> ks;
    tracer::vec3<float> ke;
    float Ns;
    bool lightsource{false};
};

struct scene {

    struct Geometry {
        std::vector<tracer::vec3<float>> vertex;
        std::vector<tracer::vec3<float>> normals;
        std::vector<tracer::vec2<float>> uv;
        std::vector<tracer::vec3<unsigned int>> face_index;
        Material object_material;
        unsigned int geomID;
    };

    std::vector<Geometry> geometry;
    std::vector<size_t> light_sources;
};

struct triangle {
    tracer::vec3<float> p0, p1, p2;
    bool has_normal;
    tracer::vec3<float> n0, n1, n2;
    Material object_material;
    size_t geomID;
    size_t primID;
};

class FlatScene {
  public:
    std::vector<triangle> triangles;
    std::vector<std::vector<triangle>> light_sources;

    FlatScene(scene s) {
        for (size_t i = 0; i < s.geometry.size(); i++) {
            auto geom = s.geometry[i];

            bool is_light = false;
            auto light_geometry = std::vector<triangle>();

            if (std::find(s.light_sources.begin(), s.light_sources.end(), i) !=
                s.light_sources.end()) {
                is_light = true;
            }

            for (size_t f = 0; f < geom.face_index.size(); f++) {
                auto face = geom.face_index[f];

                bool has_normal = false;
                tracer::vec3<float> n0, n1, n2;
                if (!geom.normals.empty()) {
                    n0 = geom.normals[face[0]];
                    n1 = geom.normals[face[1]];
                    n2 = geom.normals[face[2]];
                    has_normal = true;
                }

                triangle t =
                    {.p0 = geom.vertex[face[0]],
                     .p1 = geom.vertex[face[1]],
                     .p2 = geom.vertex[face[2]],
                     .has_normal = has_normal,
                     .n0 = n0,
                     .n1 = n1,
                     .n2 = n2,
                     .object_material = geom.object_material,
                     .geomID = i,
                     .primID = f};

                triangles.push_back(t);
                if (is_light) light_geometry.push_back(t);
            }

            if(is_light) light_sources.push_back(light_geometry);
        }
    }
};

} // namespace tracer

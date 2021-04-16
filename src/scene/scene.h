#pragma once

#include "../math/vec.h"
#include "camera.h"
#include "ray_triangle.h"

#include <algorithm>
#include <iostream>
#include <utility>
#include <variant>
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

    auto com() const -> tracer::vec3<float> {
        auto px = (p0.x + p1.x + p2.x) / 3;
        auto py = (p0.y + p1.y + p2.y) / 3;
        auto pz = (p0.z + p1.z + p2.z) / 3;
        return {px, py, pz};
    }
};

class AABB {
    tracer::vec3<float> min_point;
    tracer::vec3<float> max_point;

  public:
    AABB(tracer::vec3<float> min, tracer::vec3<float> max): min_point(min), max_point(max){};
    AABB(): min_point({0, 0, 0}), max_point({0, 0, 0}){};

    auto intersect(const tracer::ray& ray) const -> std::optional<float> {
        tracer::vec3<float> dirfrac = {1.0f / ray.dir.x, 1.0f / ray.dir.y, 1.0f / ray.dir.z};

        float t1 = (min_point.x - ray.origin.x) * dirfrac.x;
        float t2 = (max_point.x - ray.origin.x) * dirfrac.x;
        float t3 = (min_point.y - ray.origin.y) * dirfrac.y;
        float t4 = (max_point.y - ray.origin.y) * dirfrac.y;
        float t5 = (min_point.z - ray.origin.z) * dirfrac.z;
        float t6 = (max_point.z - ray.origin.z) * dirfrac.z;

        float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
        float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

        if (!(tmax < 0 || tmin > tmax))
            return tmin;
        else
            return {};
    };

    auto split() const -> std::vector<AABB> {
        auto vx = (max_point.x - min_point.x) / 2;
        auto vy = (max_point.y - min_point.y) / 2;
        auto vz = (max_point.z - min_point.z) / 2;

        return {
            AABB(
                {min_point.x, min_point.y, min_point.z},
                {max_point.x - vx, max_point.y - vy, max_point.z - vz}),
            AABB(
                {min_point.x + vx, min_point.y, min_point.z},
                {max_point.x, max_point.y - vy, max_point.z - vz}),
            AABB(
                {min_point.x + vx, min_point.y + vy, min_point.z},
                {max_point.x, max_point.y, max_point.z - vz}),
            AABB(
                {min_point.x, min_point.y + vy, min_point.z},
                {max_point.x - vx, max_point.y, max_point.z - vz}),

            AABB(
                {min_point.x, min_point.y, min_point.z + vz},
                {max_point.x - vx, max_point.y - vy, max_point.z}),
            AABB(
                {min_point.x + vx, min_point.y, min_point.z + vz},
                {max_point.x, max_point.y - vy, max_point.z}),
            AABB(
                {min_point.x + vx, min_point.y + vy, min_point.z + vz},
                {max_point.x, max_point.y, max_point.z}),
            AABB(
                {min_point.x, min_point.y + vy, min_point.z + vz},
                {max_point.x - vx, max_point.y, max_point.z})

        };
    };

    // TODO better triangle-AABB intersection
    auto is_inside(const triangle t) const -> bool {

        auto minx = std::min(std::min(t.p0.x, t.p1.x), t.p2.x);
        auto miny = std::min(std::min(t.p0.y, t.p1.y), t.p2.y);
        auto minz = std::min(std::min(t.p0.z, t.p1.z), t.p2.z);
        auto maxx = std::max(std::max(t.p0.x, t.p1.x), t.p2.x);
        auto maxy = std::max(std::max(t.p0.y, t.p1.y), t.p2.y);
        auto maxz = std::max(std::max(t.p0.z, t.p1.z), t.p2.z);

        return (minx <= max_point.x && maxx >= min_point.x) &&
               (miny <= max_point.y && maxy >= min_point.y) &&
               (minz <= max_point.z && maxz >= min_point.z);
    }

    template<typename T>
    auto is_inside(const tracer::vec3<T> c) const -> bool {
        return c.x >= min_point.x && c.x <= max_point.x && c.y >= min_point.y &&
               c.y <= max_point.y && c.z >= min_point.z && c.z <= max_point.z;
    }

    auto volume() const -> float {
        return (max_point.x - min_point.x) * (max_point.y - min_point.y) *
               (max_point.z - min_point.z);
    }
};

class BVH {
    using Nodes = std::vector<BVH>;
    using Leaves = std::vector<triangle>;

    AABB aabb;
    std::variant<Nodes, Leaves> children;

    template<typename R, typename F1, typename F2>
    struct Visitor {
        F1 f1;
        F2 f2;
        Visitor(F1 f1, F2 f2): f1(f1), f2(f2) {}

        auto operator()(Nodes const& n) -> R { return f1(n); }
        auto operator()(Leaves const& l) -> R { return f2(l); }
    };

    template<typename R, typename F1, typename F2>
    struct VisitorMut {
        F1 f1;
        F2 f2;
        VisitorMut(F1 f1, F2 f2): f1(f1), f2(f2) {}

        auto operator()(Nodes& n) -> R { return f1(n); }
        auto operator()(Leaves& l) -> R { return f2(l); }
    };

  public:
    BVH(AABB aabb, Leaves&& leaves): aabb(aabb) {
        if (aabb.volume() < 0.01 || leaves.size() < 8) {
            children = std::move(leaves);
        } else {
            Nodes new_nodes;
            for (auto const& new_aabb : aabb.split()) {
                Leaves new_leaf;
                for (auto const& triangle : leaves) {
                    if (new_aabb.is_inside(triangle)) {
                        new_leaf.push_back(triangle);
                    }
                }
                if (new_leaf.size() > 0) new_nodes.emplace_back(new_aabb, std::move(new_leaf));
            }
            children = std::move(new_nodes);
        }
    };

    static auto from_triangles(std::vector<triangle>&& triangles) -> BVH {
        auto c = triangles[0].com();
        auto minx = c.x;
        auto miny = c.y;
        auto minz = c.z;
        auto maxx = c.x;
        auto maxy = c.y;
        auto maxz = c.z;
        for (auto const& triangle : triangles) {
            auto nc = triangle.com();
            minx = (minx < nc.x) ? minx : nc.x;
            miny = (miny < nc.y) ? miny : nc.y;
            minz = (minz < nc.z) ? minz : nc.z;
            maxx = (maxx > nc.x) ? maxx : nc.x;
            maxy = (maxy > nc.y) ? maxy : nc.y;
            maxz = (maxz > nc.z) ? maxz : nc.z;
        }

        auto aabb = AABB({minx, miny, minz}, {maxx, maxy, maxz});

        return BVH(aabb, std::move(triangles));
    }

    template<typename R, typename F1, typename F2>
    auto match(F1 f1, F2 f2) const -> R {
        return std::visit(BVH::Visitor<R, F1, F2>{f1, f2}, children);
    }

    template<typename R, typename F1, typename F2>
    auto match_mut(F1 f1, F2 f2) -> R {
        return std::visit(BVH::VisitorMut<R, F1, F2>{f1, f2}, children);
    }

    auto intersect(const tracer::ray& ray, float& t, float& u, float& v) const
        -> std::optional<triangle> {
        if (!aabb.intersect(ray)) {
            return {};
        } else {
            return match<std::optional<triangle>>(
                [&](Nodes const& n) {
                    std::optional<triangle> res = {};
                    for (auto const& node : n) {
                        if (auto nn = node.intersect(ray, t, u, v)) {
                            res = nn;
                        };
                    }
                    return res;
                },
                [&](Leaves const& l) {
                    std::optional<triangle> res = {};
                    for (auto const& triangle : l) {
                        if (tracer::intersect_triangle(
                                ray.origin,
                                ray.dir,
                                triangle.p0,
                                triangle.p1,
                                triangle.p2,
                                t,
                                u,
                                v)) {
                            res = triangle;
                        }
                    }
                    return res;
                });
        }
    }

    auto oclusion(const tracer::ray& ray, float& t) const -> bool {
        if (!aabb.intersect(ray)) {
            return {};
        } else {
            return match<bool>(
                [&](Nodes const& n) {
                    for (auto const& node : n) {
                        if (node.oclusion(ray, t)) return true;
                    }
                    return false;
                },
                [&](Leaves const& l) {
                    float u, v;
                    for (auto const& triangle : l) {
                        if (tracer::intersect_triangle(
                                ray.origin,
                                ray.dir,
                                triangle.p0,
                                triangle.p1,
                                triangle.p2,
                                t,
                                u,
                                v)) {
                            return true;
                        }
                    }
                    return false;
                });
        }
    }
};

class TreeScene {
    BVH bvh;

  public:
    std::vector<std::vector<triangle>> light_sources;

    TreeScene(BVH bvh, std::vector<std::vector<triangle>> light_sources)
        : bvh(bvh), light_sources(light_sources){};

    auto intersect(const tracer::ray& ray, float& t, float& u, float& v) const
        -> std::optional<triangle> {
        return bvh.intersect(ray, t, u, v);
    };

    auto oclusion(const tracer::ray& ray, float& t) const -> bool { return bvh.oclusion(ray, t); };

    static auto from_scene(scene s) -> TreeScene {
        std::vector<triangle> triangles;
        std::vector<std::vector<triangle>> light_sources;

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

                triangle t = {
                    .p0 = geom.vertex[face[0]],
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

            if (is_light) light_sources.push_back(light_geometry);
        }

        std::cerr << "n triangles:" << triangles.size() << "\n";

        return TreeScene(BVH::from_triangles(std::move(triangles)), light_sources);
    }
};
} // namespace tracer

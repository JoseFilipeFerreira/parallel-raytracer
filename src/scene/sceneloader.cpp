#include "sceneloader.h"

#include "../math/vec.h"
#include "scene.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

tracer::scene model::loadobj(const std::string& objFileName) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> obj_materials;
    std::string err, warn;

    std::string obj_base_dir = objFileName.substr(0, objFileName.rfind('/') + 1);
    std::string mat_json_file =
        objFileName.substr(0, objFileName.rfind('.') + 1) + std::string("mat");

    auto ret = tinyobj::LoadObj(
        &attrib, &shapes, &obj_materials, &err, objFileName.c_str(), obj_base_dir.c_str(), true);

    if (!ret || !err.empty()) {
        throw std::runtime_error("TinyOBJ Error loading " + objFileName + " error: " + err);
    }

    tracer::scene scene;

    for (size_t s = 0; s < shapes.size(); ++s) {
        tracer::scene::Geometry obj;

        const tinyobj::mesh_t& obj_mesh = shapes[s].mesh;

        auto minmax_matid =
            std::minmax_element(obj_mesh.material_ids.begin(), obj_mesh.material_ids.end());

        if (*minmax_matid.first != *minmax_matid.second) {
            std::runtime_error("Warning: per-face material IDs are not supported, "
                               "materials may look "
                               "wrong."
                               " Please reexport your mesh with each material group "
                               "as an OBJ group\n");
        }

        // obj.object_material.ka

        auto mat = obj_materials[obj_mesh.material_ids[0]];
        obj.object_material.ka =
            tracer::vec3<float>(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
        obj.object_material.kd =
            tracer::vec3<float>(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
        obj.object_material.ks =
            tracer::vec3<float>(mat.specular[0], mat.specular[1], mat.specular[2]);
        obj.object_material.ke =
            tracer::vec3<float>(mat.emission[0], mat.emission[1], mat.emission[2]);
        obj.object_material.Ns = mat.shininess;

        obj.object_material.lightsource =
            (tracer::dot(obj.object_material.ke, obj.object_material.ke) > 0);

        for (size_t f = 0; f < obj_mesh.num_face_vertices.size(); ++f) {
            if (obj_mesh.num_face_vertices[f] != 3) {
                throw std::runtime_error(
                    "Non-triangle face found in " + objFileName + "-" + shapes[s].name);
            }

            tracer::vec3<unsigned int> tri_indices;
            for (size_t i = 0; i < 3; ++i) {
                const tracer::vec3<int> idx(
                    obj_mesh.indices[f * 3 + i].vertex_index,
                    obj_mesh.indices[f * 3 + i].normal_index,
                    obj_mesh.indices[f * 3 + i].texcoord_index);

                uint32_t vert_idx = obj.vertex.size();

                obj.vertex.emplace_back(
                    attrib.vertices[3 * idx.x + 0],
                    attrib.vertices[3 * idx.x + 1],
                    attrib.vertices[3 * idx.x + 2]);

                if (idx.y != uint32_t(-1)) {
                    tracer::vec3<float> n(
                        attrib.normals[3 * idx.y],
                        attrib.normals[3 * idx.y + 1],
                        attrib.normals[3 * idx.y + 2]);
                    obj.normals.push_back(normalize(n));
                }

                if (idx.z != uint32_t(-1)) {
                    obj.uv.emplace_back(
                        attrib.texcoords[2 * idx.z], attrib.texcoords[2 * idx.z + 1]);
                }

                tri_indices[i] = vert_idx;
            }
            obj.face_index.push_back(std::move(tri_indices));
        }
        scene.geometry.push_back(std::move(obj));
        if (obj.object_material.lightsource) {
            scene.light_sources.push_back(scene.geometry.size() - 1);
        }
    }
    return std::move(scene);
}

// tracer::scene::Material model::getMaterial(const std::string &name,
//                                            Json::Value &materials) {
//   if (materials.isMember(name)) {
//     auto &node = materials[name];
//     // std::cout << name << std::endl;

//     for (auto &p : node.getMemberNames()) {

//       if (p == "Type")
//         continue;

//       // std::cout << "\t" << p << std::endl;

//       if (node[p].isDouble())
//         mat.setParam(p, node[p].as<float>());
//       else if (node[p].isInt()) {
//         mat.setParam(p, node[p].as<int>());
//       } else if (node[p].isBool()) {
//         mat.setParam(p, node[p].as<bool>());
//       } else if (node[p].isString()) {
//         mat.setParam(p, node[p].asString());
//       } else if (node[p].isArray()) {
//         auto size = node[p].size();
//         if (size == 3) {
//           mat.setParam(p, tracer::vec3<float>(node[p][0].as<float>(),
//                                               node[p][1].as<float>(),
//                                               node[p][2].as<float>()));
//         } else if (size == 4) {
//           mat.setParam(p, tracer::vec4<float>(
//                               node[p][0].as<float>(), node[p][1].as<float>(),
//                               node[p][2].as<float>(),
//                               node[p][3].as<float>()));
//         }
//       }
//     }
//     return;
//   }
//   return;
// }

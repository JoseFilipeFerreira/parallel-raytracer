#pragma once

#include "scene.h"

#include <string>
#include <vector>

struct model {
    static tracer::scene loadobj(const std::string& objFileName);
    //   static tracer::scene::Material getMaterial(const std::string &name,
    //                                              Json::Value &materials);
};

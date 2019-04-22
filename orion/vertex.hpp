#pragma once

#include <orion/math.hpp>

namespace orion {

// Representation of primitive's vertex
struct Vertex {
    // position
    vec3f position;
    // normal
    vec3f normal;
    // tangent
    vec3f tangent;
    // bitangent/binormal
    vec3f bitangent;
    // texCoords
    vec2f texCoords;
};

}; // namespace orion
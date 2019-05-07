#pragma once

#include <orion/math.hpp>

namespace orion {

// Representation of primitive's vertex
struct Vertex {
    // position
    vec3f position;
    // normal
    vec3f normal;
    // texCoords
    vec2f texCoords;

    /** removed from service AD 2019 **/
    // // tangent
    // vec3f tangent;
    // // bitangent/binormal
    // vec3f bitangent;
};

}; // namespace orion
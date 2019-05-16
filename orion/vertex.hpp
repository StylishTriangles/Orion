#pragma once

#include <orion/math.hpp>

namespace orion {

// Representation of primitive's vertex
struct Vertex {
    // position
    vec3f_compact position;
    // normal
    vec3f_compact normal;
    // texCoords
    vec2f texCoords;

    /** removed from service AD 2019 **/
    // // tangent
    // vec3f_compact tangent;
    // // bitangent/binormal
    // vec3f_compact bitangent;
};

}; // namespace orion
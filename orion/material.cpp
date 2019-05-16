#include <orion/material.hpp>

namespace orion {
vec3f Material::normalBumpMap(vec3f normal, vec3f tangent, vec3f bitangent, const vec2f& uv) const {
    // normal in tangent space read from texture
    const vec3f n = normalize(tex[NORMAL].color(uv) * vec3f(2.0f) - vec3f(1.0f));

    normal.normalize();
    tangent.normalize();
    bitangent.normalize();
    // Cross product of 2 unit vector is a unit vector itself, so we just cross our fingers and hope no floating point magic happens here
    // vec3f tangent = cross(normal, vec3f(0,1,0));
    // if (!tangent.length2())
    //     tangent = cross(normal, vec3f(0,0,1));
    // vec3f bitangent = cross(normal, tangent);

    vec3f ret(
        tangent.x() * n.x() + bitangent.x() * n.y() + normal.x() * n.z(),
        tangent.y() * n.x() + bitangent.y() * n.y() + normal.y() * n.z(),
        tangent.z() * n.x() + bitangent.z() * n.y() + normal.z() * n.z()
    );

    return ret;
}

}
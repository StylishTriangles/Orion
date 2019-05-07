#include <orion/material.hpp>

namespace orion {
vec3f Material::normalBumpMap(vec3f normal, vec3f tangent, vec3f bitangent, const vec2f& uv) const {
    const vec3f n = normalize(tex[NORMAL].color(uv) * vec3f(2.0f) - vec3f(1.0f));
    // vec3f calcB = cross(normal, tangent);
    // if (dot(calcB, bitangent) < 0)
    //     tangent = -tangent;

    // std::swap(tangent, bitangent);
    normal.normalize();
    tangent = cross(normal, vec3f(0,1,0)).normalized();
    if (!tangent.length2())
        tangent = cross(normal, vec3f(0,0,1));
    bitangent = cross(normal, tangent).normalized();

    alignas(16) float res[4];
    res[0] = tangent.x() * n.x() + bitangent.x() * n.y() + normal.x() * n.z();
    res[1] = tangent.y() * n.x() + bitangent.y() * n.y() + normal.y() * n.z();
    res[2] = tangent.z() * n.x() + bitangent.z() * n.y() + normal.z() * n.z();
    res[3] = 0.0f;

    vec3f ret;
    ret.load_aligned(res);
    return ret;
    // vec3f zero(0.0f);
    // _MM_TRANSPOSE4_PS(tangent.vec, bitangent.vec, normal.vec, zero.vec);
    // return vec3f(dot(tangent,bumpNormal), dot(bitangent, bumpNormal), dot(normal, bumpNormal));

    // return vec3f(dot(tangent,bumpNormal), dot(bitangent, bumpNormal), dot(normal, bumpNormal));
    // fix tangents on mirrored vertices
    // vec3f calcB = cross(normal, tangent);
    // if (dot(calcB, bitangent) < 0)
    //     return -tangent*bumpNormal + bitangent*bumpNormal + normal*bumpNormal;
    // return tangent*bumpNormal + bitangent*bumpNormal + normal*bumpNormal;
    // vec3f calcB = cross(normal, tangent);
    // if (dot(calcB, bitangent) < 0)
    //     return vec3f(dot(-tangent,bumpNormal), dot(bitangent, bumpNormal), dot(normal, bumpNormal));
    // 
}

}
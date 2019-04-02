#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <vector>

#include <orion/math.hpp>
#include <orion/geometry.hpp>

namespace orion {

struct SolidSurface {
    vec3f color_ambient;
    vec3f color_diffuse;
    vec3f color_specular;
    float shininess, opacity;
};

// TracedMesh is a triangle mesh tailored for raytracing.
// It's based on the Mesh class and has a name distingiushing it from the original.
// For now it's very simple, but it may change as the raytracer evolves! :)
class TracedMesh
{
public:
    /*  Mesh Data  */
    std::vector<Triangle> triangles;
    SolidSurface baseColor; // on top of this will be applied textures

    // Textures are cut from this Mesh at the moment

    /*  Functions  */
    // constructor, default
    TracedMesh() = default;

    // constructor
    TracedMesh(std::vector<Triangle> triangles, SolidSurface baseColor) {
        this->triangles = triangles;
        this->baseColor = baseColor;
    }
    
    ~TracedMesh() = default;

    const Triangle* intersect(const vec3f &orig, 
                              const vec3f &dir,
                              float &t,
                              float &u,
                              float &v) const
    {
        const Triangle *ret = nullptr;
        t = 1e8;
        for (Triangle const& tri: triangles) {
            float ucurr, vcurr;
            float tcurr;
            bool section = tri.intersect(orig, dir, tcurr, ucurr, vcurr);
            if (section && tcurr < t) {
                t = tcurr;
                u = ucurr;
                v = vcurr;
                ret = &tri;
            }
        }
        return ret;
    }
};

};

#endif // ORION_MESH_HPP
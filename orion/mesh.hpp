#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <memory>
#include <vector>

#include <orion/math.hpp>
#include <orion/geometry.hpp>
#include <orion/material.hpp>

namespace orion {

// TracedMesh is a triangle mesh tailored for raytracing.
// It's based on the Mesh class and has a name distingiushing it from the original.
// For now it's very simple, but it may change as the raytracer evolves! :)
class TracedMesh
{
public:
    /*  Mesh Data  */
    std::vector<Triangle> triangles;
    std::unique_ptr<Material> pMat; // on top of this will be applied textures

    // Textures are cut from this Mesh at the moment

    /*  Functions  */
    // constructor, default
    TracedMesh() = default;

    // copy constructor performs a deep copy of TracedMesh
    TracedMesh(const TracedMesh& tm) :
        triangles( tm.triangles ),
        pMat( new Material( *tm.pMat ))
    {
        // reassign pointers because pMat changed
        assingMaterialToTriangles();
    }

    // move constructor
    TracedMesh(TracedMesh&& tm) :
        triangles( std::move(tm.triangles) ),
        pMat( std::move(tm.pMat) ) 
    {
        // no need to modify material pointers since we are just moving stuff around
    }

    // constructor
    TracedMesh(const std::vector<Triangle> &triangles, const Material &mat) :
        triangles(triangles), 
        pMat(new Material(mat))
    {
        assingMaterialToTriangles();
    }
    
    ~TracedMesh() = default;

    const Triangle* intersect(const vec3f &orig, 
                              const vec3f &dir,
                              float &t,
                              float &u,
                              float &v) const
    {
        const Triangle *ret = nullptr;
        for (Triangle const& tri: triangles) {
            float tcurr = F_INFINITY;
            float ucurr, vcurr;
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

private:
    void assingMaterialToTriangles() {
        for (Triangle& tri: this->triangles) {
            tri.pMaterial = pMat.get();
        }
    }

};

};

#endif // ORION_MESH_HPP
#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <cassert>
#include <memory>
#include <vector>

#include <orion/math.hpp>
#include <orion/geometry.hpp>
#include <orion/material.hpp>

namespace orion {

struct Vertex {
    // position
    vec3f position;
    // normal
    vec3f normal;
    // texCoords
    vec2f texCoords;
};

const unsigned int INVALID_INTERSECT_ID = ~(unsigned int)(0);

// TracedMesh is a triangle mesh tailored for raytracing.
// It's based on the Mesh class and has a name distingiushing it from the original.
// For now it's very simple, but it may change as the raytracer evolves! :)
class TracedMesh
{
public:
    /*  Mesh Data  */
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Triangle> triangles; 
    std::unique_ptr<Material> pMat; // on top of this will be applied textures

    // Textures are cut from this Mesh at the moment

    /*  Functions  */
    // constructor, default
    TracedMesh() = default;

    // copy constructor performs a deep copy of TracedMesh
    TracedMesh(const TracedMesh& tm) :
        vertices    (tm.vertices),
        indices     (tm.indices),
        triangles   (tm.triangles),
        pMat        (new Material(*tm.pMat))
    {}

    // move constructor
    TracedMesh(TracedMesh&& tm) :
        vertices    (std::move(tm.vertices)),
        indices     (std::move(tm.indices)),
        triangles   (std::move(tm.triangles)),
        pMat        (std::move(tm.pMat)) 
    {}

    // constructor
    TracedMesh( const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices, const Material &mat) :
        vertices(vertices),
        indices(indices),
        pMat(new Material(mat))
    {
        assert(indices.size()%3 == 0);
        for(unsigned int i = 0; i < this->indices.size(); i += 3)
        {
            // transform vertices and indices to triangles
            Vertex vertexes[3] = {this->vertices[this->indices[i+0]], 
                                  this->vertices[this->indices[i+1]], 
                                  this->vertices[this->indices[i+2]]};
            Triangle t = Triangle(vertexes[0].position,
                                  vertexes[1].position,
                                  vertexes[2].position);
            triangles.push_back(t);
        }
    }
    
    ~TracedMesh() = default;

    unsigned int intersect(const vec3f &orig, 
                              const vec3f &dir,
                              float &t,
                              float &u,
                              float &v) const
    {
        unsigned int retID = INVALID_INTERSECT_ID;
        for (unsigned int i = 0; i < triangles.size(); i++) {
            Triangle const& tri = triangles[i];
            float tcurr = F_INFINITY;
            float ucurr, vcurr;
            bool section = tri.intersect(orig, dir, tcurr, ucurr, vcurr);
            if (section && tcurr < t) {
                t = tcurr;
                u = ucurr;
                v = vcurr;
                retID = i;
            }
        }
        return retID;
    }

    const Material& material() const {
        return *pMat;
    }

    const Triangle& getTriangle(unsigned int triangleID) const {
        return triangles[triangleID];
    }
};

};

#endif // ORION_MESH_HPP
#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <cassert>
#include <memory>
#include <vector>

#include <orion/math.hpp>
#include <orion/geometry.hpp>
#include <orion/material.hpp>
#include <orion/avx/geometry.hpp>

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
    std::vector<PackedTriangles> triangles;
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
        // TODO: Optimize process by not constructing singulat triangles ?
        assert(indices.size()%3 == 0);
        const unsigned packSize = PackedTriangles::count;
        Triangle buffer[packSize];

        int buffer_index = 0;
        for(unsigned int i = 0; i < this->indices.size(); i += 3)
        {
            // transform vertices and indices to triangles
            Vertex vertexes[3] = {this->vertices[this->indices[i+0]], 
                                  this->vertices[this->indices[i+1]], 
                                  this->vertices[this->indices[i+2]]};
            Triangle t = Triangle(vertexes[0].position,
                                  vertexes[1].position,
                                  vertexes[2].position);
            buffer[buffer_index] = t;
            buffer_index++;

            if (buffer_index == packSize) {
                triangles.push_back(PackedTriangles(buffer, packSize));
                buffer_index %= packSize;
            }
        }
        if (buffer_index != 0) {
            triangles.push_back(PackedTriangles(buffer, buffer_index));
        }
    }
    
    ~TracedMesh() = default;

    // @returns triangleID with which ray intersects
    unsigned int intersect(const vec3f &orig, 
                           const vec3f &dir,
                           float &t,
                           float &u,
                           float &v) const
    {
        unsigned int retID = INVALID_INTERSECT_ID;
        PackedRay pr(orig, dir);
        for (unsigned int i = 0; i < triangles.size(); i++) {
            int insideID = triangles[i].intersect(pr, t, u, v);
            if (insideID >= 0) {
                retID = PackedTriangles::count * i + insideID;
            }
        }
        return retID;
    }

    const Material& material() const {
        return *pMat;
    }

    vec3f normal(unsigned int triangleID, float u, float v) const {
        return (1.0f-u-v) * vertices[3*triangleID].normal
                + u * vertices[3*triangleID+1].normal
                + v * vertices[3*triangleID+2].normal;
    }
};

};

#endif // ORION_MESH_HPP
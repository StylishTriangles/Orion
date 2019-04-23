#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <cassert>
#include <memory>
#include <vector>

#include <orion/geometry.hpp>
#include <orion/material.hpp>
#include <orion/math.hpp>
#include <orion/vertex.hpp>
#include <orion/avx/geometry.hpp>

namespace orion {

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
        triangles(packTriangles(vertices, indices)),
        pMat(new Material(mat))
    {}
    
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

    /** @brief Interpolate normal using uv values from intersection
     *  @param triangleID: id of intersected triangle
     *  @param u: distance along one edge of triangle [0.0-1.0]
     *  @param v: distance along second edge of triangle [0.0-1.0]
     *  @returns Interpolated normal
     **/
    vec3f normal(unsigned int triangleID, float u, float v) const {
        return (1.0f-u-v) * vertices[3*triangleID].normal
                + u * vertices[3*triangleID+1].normal
                + v * vertices[3*triangleID+2].normal;
    }

    /** @brief Interpolate texture uvs
     *  @param triangleID: id of intersected triangle
     *  @param u: distance along one edge of triangle [0.0-1.0]
     *  @param v: distance along second edge of triangle [0.0-1.0]
     *  @returns Interpolated texture uv at point of intersection
     **/
    vec2f texture_uv(unsigned int triangleID, float u, float v) const {
        return (1.0f-u-v) * vertices[3*triangleID].texCoords
                + u * vertices[3*triangleID+1].texCoords
                + v * vertices[3*triangleID+2].texCoords;
    }

private:
    std::vector<PackedTriangles> packTriangles(const std::vector<Vertex> &vVertices, const std::vector<unsigned int> &vIndices) {
        assert(indices.size()%3 == 0);
        const unsigned packSize = PackedTriangles::count;
        std::vector<PackedTriangles> ret;
        Triangle buffer[packSize];

        int buffer_index = 0;
        for(unsigned int i = 0; i < vIndices.size(); i += 3)
        {
            // transform vertices and indices to triangles
            Vertex vertexes[3] = {vVertices[vIndices[i+0]], 
                                  vVertices[vIndices[i+1]], 
                                  vVertices[vIndices[i+2]]};
            Triangle t = Triangle(vertexes[0].position,
                                  vertexes[1].position,
                                  vertexes[2].position);
            buffer[buffer_index] = t;
            buffer_index++;

            if (buffer_index == packSize) {
                ret.push_back(PackedTriangles(buffer, packSize));
                buffer_index %= packSize;
            }
        }
        if (buffer_index != 0) {
            ret.push_back(PackedTriangles(buffer, buffer_index));
        }
        return ret;
    }
};

};

#endif // ORION_MESH_HPP
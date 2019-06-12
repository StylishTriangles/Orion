#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <cassert>
#include <memory>
#include <vector>

#include <orion/geometry.hpp>
#include <orion/material.hpp>
#include <orion/math.hpp>
#include <orion/vertex.hpp>
// #include <orion/bvh.hpp>
#include <orion/avx/geometry.hpp>
#include <orion/avx/sbvh.hpp>

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
    // BVH<Triangle> mT2;
    SBVH mTree;
    unsigned mID;
    float mSurfaceArea;

    // Textures are cut from this Mesh at the moment

    /*  Functions  */
    // constructor, default
    TracedMesh() = default;

    // copy constructor performs a deep copy of TracedMesh
    TracedMesh(const TracedMesh& tm) :
        vertices    (tm.vertices),
        indices     (tm.indices),
        triangles   (tm.triangles),
        pMat        (new Material(*tm.pMat)),
        mTree       (tm.mTree),
        mID         (tm.mID),
        mSurfaceArea(tm.mSurfaceArea)
    {}

    // // move constructor
    // TracedMesh(TracedMesh&& tm) :
    //     vertices    (std::move(tm.vertices)),
    //     indices     (std::move(tm.indices)),
    //     triangles   (std::move(tm.triangles)),
    //     pMat        (std::move(tm.pMat)) 
    // {}

    // constructor
    TracedMesh( const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices, const Material &mat) :
        vertices(vertices),
        indices(indices),
        triangles(packTriangles(vertices, indices)),
        pMat(new Material(mat)),
        mTree(vertices, indices)
    {
        static unsigned id = 0;
        mID = id++;
        calcSurfaceArea();
        // std::vector<Triangle> tris;
        // for(unsigned int i = 0; i < indices.size(); i += 3)
        // {
        //     // transform vertices and indices to triangles
        //     Vertex vertexes[3] = {vertices[indices[i+0]], 
        //                           vertices[indices[i+1]], 
        //                           vertices[indices[i+2]]};
        //     Triangle t = Triangle(vertexes[0].position,
        //                           vertexes[1].position,
        //                           vertexes[2].position);
        //     tris.push_back(t);
        // }
        // mT2 = std::move(BVH<Triangle>(std::move(tris)));
    }
    
    ~TracedMesh() = default;

    // @returns triangleID with which ray intersects
    unsigned int intersect(const vec3f &orig, 
                           const vec3f &dir,
                           float &t,
                           float &u,
                           float &v) const
    {
        return mTree.intersect(
            orig,
            dir,
            t,
            u,
            v
        );
        // unsigned int retID = INVALID_INTERSECT_ID;
        // PackedRay pr(orig, dir);
        // for (unsigned int i = 0; i < triangles.size(); i++) {
        //     int insideID = triangles[i].intersect(pr, t, u, v);
        //     if (insideID >= 0) {
        //         retID = PackedTriangles::count * i + insideID;
        //     }
        // }
        // return retID;
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
        return (1.0f-u-v) * vertices[indices[3*triangleID]].normal
                + u * vertices[indices[3*triangleID+1]].normal
                + v * vertices[indices[3*triangleID+2]].normal;
    }
    
    // vec3f tangent(unsigned int triangleID, float u, float v) const {
    //     return (1.0f-u-v) * vertices[indices[3*triangleID]].tangent
    //             + u * vertices[indices[3*triangleID+1]].tangent
    //             + v * vertices[indices[3*triangleID+2]].tangent;
    // }

    // vec3f bitangent(unsigned int triangleID, float u, float v) const {
    //     return (1.0f-u-v) * vertices[indices[3*triangleID]].bitangent
    //             + u * vertices[indices[3*triangleID+1]].bitangent
    //             + v * vertices[indices[3*triangleID+2]].bitangent;
    // }

    /** @brief Interpolate texture uvs
     *  @param triangleID: id of intersected triangle
     *  @param u: distance along one edge of triangle [0.0-1.0]
     *  @param v: distance along second edge of triangle [0.0-1.0]
     *  @returns Interpolated texture uv at point of intersection
     **/
    vec2f texture_uv(unsigned int triangleID, float u, float v) const {
        return (1.0f-u-v) * vertices[indices[3*triangleID]].texCoords
                + u * vertices[indices[3*triangleID+1]].texCoords
                + v * vertices[indices[3*triangleID+2]].texCoords;
    }

    unsigned triangleCount() const {
        return indices.size()/3;
    }

    vec3f lowerBound() const {
        return mTree.lowerBound();
    }
    vec3f upperBound() const {
        return mTree.upperBound();
    }

    // Returns the surface area
    float surfaceArea() const {
        return mSurfaceArea;
    }

    /**
     *  @param rng: random number generator with overloaded operator(void)
     *  @param point (out): randomly generated point on surface
     *  @param bias (out): points may not conform to uniform distribution, algorithm picks triangles at random, so
     *      small triangle is as likely to be chosen as a big triangle. Bias of 1.0 means that point was chosen uniformly,
     *      Bias < 1.0 means that returned point was more likely to be chosen than it should've been
     *      Bias > 1.0 means that returned point was less likely to be chosen
     */
    template<typename RandomNumberGenerator>
    void randomPointOnSurface(RandomNumberGenerator &rng, vec3f& point, float& bias) const {
        unsigned random_index = unsigned(rng())%triangleCount();
        Triangle tri = triangleAt(random_index);
        bias = (tri.surfaceArea()/surfaceArea()) * triangleCount();
        tri.randomPointOnSurface(rng, point);
    }

    Triangle triangleAt(unsigned index) const {
        index = index * 3;
        return Triangle(
            vertices[indices[index]].position,
            vertices[indices[index+1]].position,
            vertices[indices[index+2]].position
        );
    }

private:

    void calcSurfaceArea() {
        float res = 0;
        for (unsigned i = 0; i < indices.size(); i+=3) {
            Triangle tri(
                vertices[indices[i]].position,
                vertices[indices[i+1]].position,
                vertices[indices[i+2]].position
            );
            res += tri.surfaceArea();
        }
        mSurfaceArea = res;
    }

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
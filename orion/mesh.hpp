#ifndef ORION_MESH_HPP
#define ORION_MESH_HPP

#include <cassert>
#include <memory>
#include <vector>

#include <orion/geometry.hpp>
#include <orion/material.hpp>
#include <orion/math.hpp>
#include <orion/vertex.hpp>
#include <orion/bvh.hpp>
#include <orion/avx/geometry.hpp>
#include <orion/avx/sbvh.hpp>

namespace orion {

const unsigned int INVALID_INTERSECT_ID = ~(unsigned int)(0);

// TracedMesh is a triangle mesh tailored for raytracing.
// It's based on the Mesh class and has a name distingiushing it from the original.
// For now it's very simple, but it may change as the raytracer evolves! :)
class TracedMesh : public Primitive
{
public:
    /*  Mesh Data  */
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<PackedTriangles> triangles;
    std::unique_ptr<Material> pMat; // on top of this will be applied textures
    BVH<Triangle> mT2;
    // SBVH mTree;

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
        // mTree       (tm.mTree)
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
        pMat(new Material(mat))
        // mTree(vertices, indices)
    {
        std::vector<Triangle> tris;
        for(unsigned int i = 0; i < indices.size(); i += 3)
        {
            // transform vertices and indices to triangles
            Vertex vertexes[3] = {vertices[indices[i+0]], 
                                  vertices[indices[i+1]], 
                                  vertices[indices[i+2]]};
            Triangle t = Triangle(vertexes[0].position,
                                  vertexes[1].position,
                                  vertexes[2].position);
            t.id = i/3;
            tris.push_back(t);
        }
        mT2 = std::move(BVH<Triangle>(std::move(tris)));
    }
    
    ~TracedMesh() = default;

    // @returns triangleID with which ray intersects
    // unsigned int intersect(const vec3f &orig, 
    //                        const vec3f &dir,
    //                        float &t,
    //                        float &u,
    //                        float &v) const
    // {
        // return mTree.intersect(
        //     orig,
        //     dir,
        //     t,
        //     u,
        //     v
        // );
        // unsigned int retID = INVALID_INTERSECT_ID;
        // PackedRay pr(orig, dir);
        // for (unsigned int i = 0; i < triangles.size(); i++) {
        //     int insideID = triangles[i].intersect(pr, t, u, v);
        //     if (insideID >= 0) {
        //         retID = PackedTriangles::count * i + insideID;
        //     }
        // }
        // return retID;
    // }

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

    int triangleCount() const {
        return indices.size()/3;
    }

    vec3f lowerBound() const {
        return mT2.lowerBound();
    }
    vec3f upperBound() const {
        return mT2.upperBound();
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

template <>
class Intersector<TracedMesh> {
public:
    template <unsigned size> 
    class Intersection;

    template<unsigned size>
    void intersect(
        const PackedRay<size>& r,
        const TracedMesh& mesh,
        Intersection<size>& res
    ) {
        Intersector<BVH<Triangle>> inter;
        Intersector<BVH<Triangle>>::Intersection<size> tmp_res;
        for (unsigned i = 0; i < size; i++) {
            tmp_res[i].t = res[i].t;
        }
        inter.intersect(r, mesh.mT2, tmp_res);
        for (unsigned i = 0; i < size; i++) {
            if (tmp_res[i].t < res[i].t) {
                res[i].t = tmp_res[i].t;
                res[i].uv = vec2f(tmp_res[i].u, tmp_res[i].v);
                res[i].pMesh = &mesh;
                res[i].triangleID = tmp_res[i].triangleID;
            }
        }

    }
};

template <>
class Intersector<TracedMesh>::Intersection<1> {
public:
    Intersection() : pMesh(nullptr), t(F_INFINITY) {}

    vec3f normal() const { 
        // if (material().hasBumpMap())
            // return material().normalBumpMap(surfaceNormal(), pMesh->tangent(triangleID, uv[0], uv[1]), pMesh->bitangent(triangleID, uv[0], uv[1]), texture_uv());
        return surfaceNormal();
    }
    vec3f surfaceNormal() const { return pMesh->normal(triangleID, uv[0], uv[1]); }
    vec2f texture_uv() const { return pMesh->texture_uv(triangleID, uv[0], uv[1]); }
    const Material& material() const { return pMesh->material(); }
    bool intersected() const { return pMesh != nullptr; }
    
    /** Members **/
    const TracedMesh* pMesh;
    unsigned int triangleID;
    vec2f uv;
    float t;
};

template <unsigned size>
class Intersector<TracedMesh>::Intersection {
public:
    public:
    Intersector<TracedModel>::Intersection<1>& operator[] (unsigned index) {
        return mInt[index];
    }

    const Intersector<TracedModel>::Intersection<1>& operator[] (unsigned index) const {
        return mInt[index];
    }
private:
    Intersector<TracedModel>::Intersection<1> mInt[size];
};

}; // namespace orion

#endif // ORION_MESH_HPP
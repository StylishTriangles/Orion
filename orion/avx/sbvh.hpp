#pragma once

#include <memory>
#include <vector>

#include <orion/math.hpp>
#include <orion/vertex.hpp>
#include <orion/avx/geometry.hpp>
#include <orion/avx/math.hpp>

namespace orion {

// std::pair<int, int> intersectionCount();

// Provisional structure, not meant for storage!
struct SBVHTriangle {
    SBVHTriangle(
        const Vertex& v0,
        const Vertex& v1,
        const Vertex& v2,
        unsigned originalID
    ) : originalID(originalID)
    {
        // pVert[0] = v0;
        // pVert[1] = v1;
        // pVert[2] = v2;

        vec3f minPos = min(v0.position, min(v1.position, v2.position));
        vec3f maxPos = max(v0.position, max(v1.position, v2.position));

        bv = AABB(minPos, maxPos);
    }

    vec3f centroid() const { return bv.centroid(); }

    // triangle's vertices
    // const Vertex* pVert[3];
    AABB bv;
    // ID in the original vector of indices / vector of vertices combo
    unsigned originalID;

};

struct BuildBVHNode {
    BuildBVHNode() = default;

    BuildBVHNode(const BuildBVHNode& ref) :
        bv(ref.bv),
        isLeaf(ref.isLeaf),
        begin(ref.begin),
        end(ref.end)
    {
        if (!isLeaf) {
            children[0] = std::unique_ptr<BuildBVHNode>(new BuildBVHNode(*(ref.children[0])));
            children[1] = std::unique_ptr<BuildBVHNode>(new BuildBVHNode(*(ref.children[1])));
        }
    }
    // Bounding volume
    AABB bv;
    // Check if this node is a leaf
    bool isLeaf;
    // Children - nullptr == no child
    std::unique_ptr<BuildBVHNode> children[2];
    // triangles are saved as range in triangle array
    unsigned begin, end;
};

struct SBVHNode {
    // bounding volmes of children nodes
    PackedAABB bv;
    SBVHNode* children[8];
};

// Shallow Bounding Volume Hierarchy implementation using AVX
// Motivation: Because the last step is to pack vertexes into triangles and those into packed triangles
// I have decided to create a structure that only accelerates meshes made out of triangles and 
// the representation of triangles is implicitly defined across orion
class SBVH {
public:

    // Construction strategy of the SBVH
    struct Strategy {
        enum Algorithm {
            // Find median Primitive and split there
            MEDIAN,
            // Split in the middle of axis
            MIDDLE,
            // Surface Area Heuristics
            SAH,
        };
        /** @brief Construct a SBVH building strategy
         *  @param split_rounding: On split multiples of this amount will go to the left bucket (must be >= 1)
         *  @param algo: Algorithm to be used when splitting to create children nodes
         *  @param split_candidates: How many split points will be considered (Only matters when SAH is chosen)
         *      
         *  Parameter explanation: 
         *  - split_rounding: ex. splitting 14 triangles sub-mesh would reult in a 7-7 split 
         *      however with split_rounding set to 8 this would be a 8-6 split. 
         *      This could be useful with AVX since intersections with 1 triangle take as much time as intersections with 8.
         **/
        Strategy(
            unsigned split_rounding,
            Algorithm algo,
            unsigned split_candidates,
            unsigned leaf_triangles = PackedTriangles::count
        ) : split_rounding(split_rounding),
            algo(algo),
            split_candidates(split_candidates),
            leaf_triangles(leaf_triangles) // End recursive process if no more than this many triangles are present
        {}
            
        // Split elements in a way in which into one bucket goes k * split_rounding.
        unsigned split_rounding;
        // Algorithm used for tree construction
        Algorithm algo;
        // How many split positions are we considering
        unsigned split_candidates;
        // How many triangles can be in a leaf at most
        unsigned leaf_triangles;

        /** constants for SAH **/
        // traverse cost relative to intersection cost
        static constexpr float traverseCost = 1; 
        // cost of intersection with ONE triangle
        static constexpr float intersectCost = 1; 
    };

    SBVH() = default;

    SBVH(const SBVH& ref) :
        mRoot(new BuildBVHNode(*ref.mRoot)),
        mVertices(ref.mVertices),
        mIndices(ref.mIndices),
        mTriangles(ref.mTriangles)
    {}

    // Construct Shallow Bounding Volume Hierarchy
    SBVH(
        const std::vector<Vertex>& vertices, 
        const std::vector<unsigned>& indices, 
        Strategy strategy = Strategy(8, SBVH::Strategy::MEDIAN, 12, 4)
    );

    // @returns triangleID with which ray intersects
    // Parameters are updated on successfull intersection in which t' < t
    unsigned int intersect(
        const vec3f &orig, 
        const vec3f &dir,
        float &t,
        float &u,
        float &v
    ) const;

    vec3f lowerBound() const {
        if (mRoot != nullptr)
            return mRoot->bv.lowerBound();
        return vec3f(0.0f);
    }
    vec3f upperBound() const {
        if (mRoot != nullptr)
            return mRoot->bv.upperBound();
        return vec3f(0.0f);
    }

private:
    std::vector<SBVHTriangle> toTriangles(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) const;

    std::unique_ptr<BuildBVHNode> recursiveConstruct(const Strategy& strategy, std::vector<SBVHTriangle> &triangles, unsigned begin, unsigned end) const;

    unsigned int innerIntersect(
        const vec3f &orig, 
        const vec3f &dir,
        float &t,
        float &u,
        float &v,
        BuildBVHNode *node
    ) const;

    Triangle triangleAt(unsigned index) const;

    // nodes representing SBVH tree
    std::vector<SBVHNode> nodes;

    // Provisional data
    std::unique_ptr<BuildBVHNode> mRoot;
    std::vector<Vertex> mVertices;
    std::vector<unsigned> mIndices;
    std::vector<SBVHTriangle> mTriangles;
}; // Class SBVH

}; // namespace orion
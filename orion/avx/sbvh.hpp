#pragma once

#include <vector>

#include <orion/math.hpp>
#include <orion/vertex.hpp>
#include <orion/avx/geometry.hpp>
#include <orion/avx/math.hpp>

namespace orion {

struct SBVHTriangle {
    unsigned originalID;
};

struct BuildBVHNode {
    // Bounding volume
    AABB bv;
    // Check if this node is a leaf
    bool isLeaf;
    // Children - nullptr == no children
    BuildBVHNode* children[2];
    // triangles are saved in packs into leaf nodes
    std::vector<SBVHTriangle> triangles;
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

    // Construction strategy of the SBVH
    struct Strategy {
        enum Algorithm {
            // Use Surface Area Heuristics
            SAH,
        };
        /** @brief Construct a SBVH building strategy
         *  @param split_rounding: On split multiples of this amount will go to the left bucket (must be >= 1)
         *  @param algo: Algorithm to be used when splitting to create children nodes
         *      
         *  Parameter explanation: 
         *  - split_rounding: ex. splitting 14 triangles sub-mesh would reult in a 7-7 split 
         *      however with split_rounding set to 8 this would be a 8-6 split. 
         *      This could be useful with AVX since intersections with 1 triangle take as much time as intersections with 8.
         **/
        Strategy(
            unsigned split_rounding,
            Algorithm algo
        ) : split_rounding(split_rounding),
            algo(algo) 
        {}
            
        unsigned split_rounding;
        Algorithm algo;
    };

    // Construct Shallow Bounding Volume Hierarchy
    SBVH(
        const std::vector<Vertex>& vertices, 
        const std::vector<unsigned>& indices, 
        Strategy strategy = Strategy(8, SBVH::Strategy::SAH)
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

private:
    std::vector<SBVHTriangle> toTriangles(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) const;

    // nodes representing SBVH tree
    std::vector<SBVHNode> nodes;
}; // Class SBVH

}; // namespace orion
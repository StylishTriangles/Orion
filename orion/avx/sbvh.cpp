#include <orion/avx/sbvh.hpp>

namespace orion {

/** Shallow Bounding Volume Hierarchy is contructed by collapsing a binary BVH
 *  IMPORTANT: vertices and indices vectors must not be changed throughout the construction process,
 *      especially they must not be deleted at any point.
 */
SBVH::SBVH(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices, Strategy strategy)
{
    std::vector<SBVHTriangle> triangles = toTriangles(vertices, indices);
}

unsigned int SBVH::intersect(
    const vec3f &orig, 
    const vec3f &dir,
    float &t,
    float &u,
    float &v
) const {
    
}

std::vector<SBVHTriangle> SBVH::toTriangles(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) const {
    std::vector<SBVHTriangle> triangles;
    for (int i = 0; i < indices.size(); i += 3) {
        SBVHTriangle tri;
        tri.originalID = i/3;
        triangles.push_back(tri);
    }
    return triangles;
}

};
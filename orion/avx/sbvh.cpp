#include <algorithm>

#include <orion/avx/sbvh.hpp>

namespace orion {

// Dirty hack for testing
static int triInt = 0;
static int BBInt = 0;
std::pair<int, int> intersectionCount() {
    return {BBInt, triInt};
}

/** Shallow Bounding Volume Hierarchy is contructed by collapsing a binary BVH
 *  IMPORTANT: vertices and indices vectors must not be changed throughout the construction process,
 *      especially they must not be deleted at any point.
 */
SBVH::SBVH(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices, Strategy strategy) :
    mVertices(vertices),
    mIndices(indices)
{
    mTriangles = toTriangles(vertices, indices);
    mRoot = recursiveConstruct(strategy, mTriangles, 0, mTriangles.size());
}

unsigned int SBVH::intersect(
    const vec3f &orig, 
    const vec3f &dir,
    float &t,
    float &u,
    float &v
) const {
    return innerIntersect(orig, dir, t, u, v, mRoot.get());
}

unsigned int SBVH::innerIntersect(
    const vec3f &orig, 
    const vec3f &dir,
    float &t,
    float &u,
    float &v,
    BuildBVHNode *node
) const {
    const unsigned INVALID_RESULT = ~(unsigned)(0);

    // Check if ray intersects node's bounding volume
    float tmpa = 0, tmpb = 0;
    // BBInt++;
    if (!node->bv.intersect(orig, dir, tmpa, tmpb)) {
        return INVALID_RESULT;
    }

    if (node->isLeaf) {
        unsigned intersectID = INVALID_RESULT;
        for (unsigned i = node->begin; i < node->end; i++) {
            // triInt++;
            Triangle tri = triangleAt(i);
            bool intersected = tri.intersect(orig, dir, t, u, v);
            if (intersected)
                intersectID = mTriangles[i].originalID;
        }
        return intersectID;
    } else {
        unsigned r0 = innerIntersect(
            orig,
            dir,
            t,
            u,
            v,
            node->children[0].get()
        );
        unsigned r1 = innerIntersect(
            orig,
            dir,
            t,
            u,
            v,
            node->children[1].get()
        );
        if (r1 != INVALID_RESULT)
            return r1;
        return r0;
    }
}

Triangle SBVH::triangleAt(unsigned index) const {
    index = mTriangles[index].originalID * 3;
    return Triangle(
        mVertices[mIndices[index]].position,
        mVertices[mIndices[index+1]].position,
        mVertices[mIndices[index+2]].position
    );
}

std::vector<SBVHTriangle> SBVH::toTriangles(const std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) const {
    std::vector<SBVHTriangle> triangles;
    for (unsigned i = 0; i < indices.size(); i += 3) {
        SBVHTriangle tri(
            vertices[indices[i]],
            vertices[indices[i+1]],
            vertices[indices[i+2]],
            i/3
        );
        triangles.push_back(tri);
    }
    return triangles;
}

// Bucket info structer for SAH data
struct SAHBucketInfo {
    int count = 0;
    AABB bounds;
};

// This will modify the triangles vector!
std::unique_ptr<BuildBVHNode> SBVH::recursiveConstruct(const Strategy& strategy, std::vector<SBVHTriangle> &triangles, unsigned begin, unsigned end) const {
    std::unique_ptr<BuildBVHNode> ret(new BuildBVHNode);

    // Find the bounding volume of all triangles on current level
    AABB boundAll;
    for (unsigned i = begin; i < end; i++) {
        boundAll = AABB(boundAll, triangles[i].bv);
    }
    unsigned nTriangles = end-begin;
    // save common settings
    ret->begin = begin;
    ret->end = end;
    ret->bv = boundAll;
    // Finish recursion if there are as many triangles in leaf as wanted
    if (nTriangles <= strategy.leaf_triangles) {
        ret->isLeaf = true;
        return ret;
    }

    // Pick axis on which we will split
    int axis = boundAll.maximumExtent();
    
    SBVHTriangle* pmid;
    unsigned newMid;

    switch (strategy.algo) {
    case Strategy::MEDIAN:
        std::nth_element(
            &triangles[begin],
            &triangles[(begin+end)/2],
            &triangles[end],
            [&](const SBVHTriangle &lhs, const SBVHTriangle &rhs) {
                return boundAll.offset(lhs.centroid())[axis] < boundAll.offset(rhs.centroid())[axis];
        });
        newMid = (begin+end)/2;
        ret->isLeaf = false;
        ret->children[0] = recursiveConstruct(strategy, triangles, begin, newMid);
        ret->children[1] = recursiveConstruct(strategy, triangles, newMid, end);
        break;
    case Strategy::MIDDLE:
        pmid = std::partition(&triangles[begin], &triangles[end],
            [&](const SBVHTriangle &tri) {
                return boundAll.offset(tri.centroid())[axis] <= 0.5f;
            }
        );
        if (pmid == &triangles[begin] || pmid == &triangles[end]) {
            ret->isLeaf = true;
            return ret;
        } else {
            newMid = pmid - &triangles[0];
            ret->isLeaf = false;
            ret->children[0] = recursiveConstruct(strategy, triangles, begin, newMid);
            ret->children[1] = recursiveConstruct(strategy, triangles, newMid, end);
        }
        break;
    case Strategy::SAH: // fallthrough, SAH is the default algorithm
    default:
        // Create empty buckets
        const unsigned &nBuckets = strategy.split_candidates;
        SAHBucketInfo buckets[nBuckets];

        // initialize buckets structure
        for (unsigned i = begin; i < end; i++) {
            unsigned b = nBuckets * boundAll.offset(triangles[i].centroid())[axis];
            if (b == nBuckets)
                b--;
            buckets[b].count++;
            buckets[b].bounds = AABB(buckets[b].bounds, triangles[i].bv);
        }

        // calculate cost of splitting after each bucket
        float cost[nBuckets - 1];
        AABB b0;
        unsigned count0 = 0;
        for (unsigned i = 0; i < nBuckets - 1; i++) {
            AABB b1;
            unsigned count1 = 0;

            b0 = AABB(b0, buckets[i].bounds);
            count0 += buckets[i].count;
            for (unsigned j = i+1; j < nBuckets; j++) {
                b1 = AABB(b1, buckets[j].bounds);
                count1 += buckets[j].count;
            }
            cost[i] = strategy.traverseCost + strategy.intersectCost * (count0 * b0.surfaceArea() +
                                count1 * b1.surfaceArea()) / boundAll.surfaceArea();
        }

        // find bucket after which we should split
        float minCost = cost[0];
        unsigned minCostSplitBucket = 0;
        for (unsigned i = 1; i < nBuckets - 1; i++) {
            if (cost[i] < minCost) {
                minCost = cost[i];
                minCostSplitBucket = i;
            }
        }

        float leafCost = nTriangles * strategy.intersectCost;
        // Split the triangles at point with minimum cost or construct leaf
        if (minCost >= leafCost) {
            // construct leaf
            ret->isLeaf = true;
        } else {
            // split and call
            pmid = std::partition(&triangles[begin], &triangles[end],
                [&](const SBVHTriangle &tri) {
                    unsigned b = nBuckets * boundAll.offset(tri.centroid())[axis];
                    if (b == nBuckets) b--;
                    return b <= minCostSplitBucket;
                }
            );
            newMid = pmid - &triangles[0];
            ret->isLeaf = false;
            ret->children[0] = recursiveConstruct(strategy, triangles, begin, newMid);
            ret->children[1] = recursiveConstruct(strategy, triangles, newMid, end);
        }

    }
    return ret;
}

// /** Split triangles v
//  * 
//  **/
// unsigned int SBVH::chooseSplitPoint(const std::vector<SBVHTriangle> &triangles, const Strategy& strategy, AABB& vol1, AABB& vol2) const {

// }

};

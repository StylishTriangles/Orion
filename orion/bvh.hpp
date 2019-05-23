/** 
 * Generic Bounding Volume Hierarchy
 **/

#pragma once

#include <array>
#include <vector>
#include <orion/interfaces.hpp>

namespace orion {

/**
 *  @tparam Element: must fulfill the Primitive interface
 */
template <class Element>
struct BVHNode {
    BVHNode() = default;

    BVHNode(
        const AABB& bv,
        unsigned begin,
        unsigned end 
    ) : bv(bv), begin(begin), end(end)
    {}

    BVHNode(const BVHNode& ref) :
        bv(ref.bv),
        childID(ref.childID),
        begin(ref.begin),
        end(ref.end)
    {}

    BVHNode(const BVHNode&& ref) :
        bv(std::move(ref.bv)),
        childID(std::move(ref.childID)),
        begin(ref.begin),
        end(ref.end)
    {}

    // Bounding volume
    AABB bv;
    // Children IDs
    std::array<unsigned, 2> childID;
    // Elements are saved as range in triangle array
    unsigned begin; // first bit is reserved as a leaf indicator
    unsigned end;

    // Child nodes with this ID are treated as invalid
    static constexpr unsigned invalidID = unsigned(-1);
    // Check if this node is a leaf
    constexpr bool isLeaf() const { return childID[0] == invalidID; }
    // set this node as a leaf
    void setLeaf() {
        childID[0] = invalidID;
    }
    // return the bounding volume of node
    const AABB& boundingBox() const {return bv;}
};

/**
 *  @tparam Element: Contained type, must fulfill the Primitive interface
 */
template <class Element>
class BVH : public Primitive {
public:
    // Construction strategy of the BVH
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
         *  - split_rounding: ex. splitting 14 primitives sub-mesh would reult in a 7-7 split 
         *      however with split_rounding set to 8 this would be a 8-6 split. 
         *      This could be useful with AVX since intersections with 1 triangle take as much time as intersections with 8.
         **/
        Strategy(
            unsigned split_rounding,
            Algorithm algo,
            unsigned split_candidates,
            unsigned leaf_primitives
        ) : split_rounding(split_rounding),
            algo(algo),
            split_candidates(split_candidates),
            leaf_primitives(leaf_primitives) // End recursive process if no more than this many primitives are present
        {}
            
        // Split elements in a way in which into one bucket goes k * split_rounding.
        unsigned split_rounding;
        // Algorithm used for tree construction
        Algorithm algo;
        // How many split positions are we considering
        unsigned split_candidates;
        // How many primitives can be in a leaf at most
        unsigned leaf_primitives;

        /** constants for SAH **/
        // traverse cost relative to intersection cost
        static constexpr float traverseCost = 1; 
        // cost of intersection with ONE triangle
        static constexpr float intersectCost = 1; 
    };

    class ElementIterator {
        ElementIterator(unsigned eindex) : eindex(eindex) {}
        ElementIterator(ElementIterator const&) = default;
        ElementIterator operator++() {eindex++; return *this;}
        ElementIterator operator++(int) {
            ElementIterator temp(*this);
            eindex++;
            return temp;
        }
        Element& operator*() {return mPrimitives[eindex];}
        const Element& operator*() const {return mPrimitives[eindex];}

        Element* operator->() {return &mPrimitives[eindex];}
        const Element* operator->() const {return &mPrimitives[eindex];}
    private:
        unsigned eindex;
    };

    class NodeIterator {
    public:
        NodeIterator(unsigned index) : index(index) {}
        NodeIterator leftChild() const {
            return NodeIterator(mNodes[index].childID[0]);
        }
        NodeIterator rightChild() const {
            return NodeIterator(mNodes[index].childID[1]);
        }
        bool isLeaf() const {
            return mNodes[index].isLeaf();
        }
        ElementIterator elementsBegin() const {
            return ElementIterator(mNodes[index].begin);
        }
        ElementIterator elementsEnd() const {
            return ElementIterator(mNodes[index].end);
        }
        BVHNode<Element>& operator*() {return mNodes[index];}
        const BVHNode<Element>& operator*() const {return mNodes[index];}

        BVHNode<Element>* operator->() {return &mNodes[index];}
        const BVHNode<Element>* operator->() const {return &mNodes[index];}
    private:
        unsigned index;
    };

    BVH() = default;

    BVH(const BVH& ref) = default;

    BVH(const BVH&& ref) :
        mNodes(std::move(ref.mNodes)),
        mPrimitives(std::move(ref.mPrimitives))
    {}

    // Construct Shallow Bounding Volume Hierarchy
    BVH(
        const std::vector<Element>& primitives,
        Strategy strategy = Strategy(8, Strategy::MEDIAN, 12, 8)
    );

    vec3f lowerBound() const {
        return this->boundingBox().lowerBound();
    }

    vec3f upperBound() const {
        return this->boundingBox().upperBound();
    }

    AABB boundingBox() const {
        if (!mNodes.empty())
            return mNodes.front().bv;
        return AABB();
    }

    unsigned primitiveCount() const {
        return mPrimitives.size();
    }

    BVH& operator= (BVH&&) = default;

    NodeIterator root() const {
        return NodeIterator(0);
    }
private:
    unsigned recursiveConstruct(const Strategy& strategy, std::vector<Element> &primitives, unsigned begin, unsigned end);

    // store nodes in a vector for better memory access patterns
    std::vector<BVHNode<Element>> mNodes;
    // vector of provided elements
    std::vector<Element> mPrimitives;
};

template <class Element>
BVH<Element>::BVH(
    const std::vector<Element>& primitives,
    Strategy strategy
) : mPrimitives(primitives)
{
    recursiveConstruct(strategy, mPrimitives, 0, primitives.size());
    mNodes.shrink_to_fit();
}

// Bucket info structer for SAH data
struct SAHBucketInfo {
    int count = 0;
    AABB bounds;
};

template <class Element>
inline unsigned BVH<Element>::recursiveConstruct(const Strategy& strategy, std::vector<Element> &primitives, unsigned begin, unsigned end) {
    // Find the bounding volume of all primitives on current level
    AABB boundAll;
    for (unsigned i = begin; i < end; i++) {
        boundAll = AABB(boundAll, primitives[i].boundingBox());
    }
    unsigned nPrimitives = end-begin;
    // save common settings
    mNodes.emplace_back(boundAll, begin, end);
    BVHNode<Element>& ret = mNodes.back();
    unsigned retID = mNodes.size()-1;

    // Finish recursion if there are as many primitives in leaf as wanted
    if (nPrimitives <= strategy.leaf_primitives) {
        ret.setLeaf();
        return retID;
    }

    // Pick axis on which we will split
    int axis = boundAll.maximumExtent();
    
    Element* pmid;
    unsigned newMid;

    switch (strategy.algo) {
    case Strategy::MEDIAN:
        std::nth_element(
            &primitives[begin],
            &primitives[(begin+end)/2],
            &primitives[end],
            [&](const Element &lhs, const Element &rhs) {
                return boundAll.offset(lhs.centroid())[axis] < boundAll.offset(rhs.centroid())[axis];
        });
        newMid = (begin+end)/2;
        ret.childID[0] = recursiveConstruct(strategy, primitives, begin, newMid);
        ret.childID[1] = recursiveConstruct(strategy, primitives, newMid, end);
        break;
    case Strategy::MIDDLE:
        pmid = std::partition(&primitives[begin], &primitives[end],
            [&](const Element &e) {
                return boundAll.offset(e.centroid())[axis] <= 0.5f;
            }
        );
        if (pmid == &primitives[begin] || pmid == &primitives[end]) {
            ret.setLeaf();
            return retID;
        } else {
            newMid = pmid - &primitives[0];
            ret.childID[0] = recursiveConstruct(strategy, primitives, begin, newMid);
            ret.childID[1] = recursiveConstruct(strategy, primitives, newMid, end);
        }
        break;
    case Strategy::SAH: // fallthrough, SAH is the default algorithm
    default:
        // Create empty buckets
        const unsigned &nBuckets = strategy.split_candidates;
        SAHBucketInfo buckets[nBuckets];

        // initialize buckets structure
        for (unsigned i = begin; i < end; i++) {
            unsigned b = nBuckets * boundAll.offset(primitives[i].centroid())[axis];
            if (b == nBuckets)
                b--;
            buckets[b].count++;
            buckets[b].bounds = AABB(buckets[b].bounds, primitives[i].boundingBox());
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

        float leafCost = nPrimitives * strategy.intersectCost;
        // Split the primitives at point with minimum cost or construct leaf
        if (minCost >= leafCost) {
            // construct leaf
            ret.setLeaf();
        } else {
            // split and call
            pmid = std::partition(
                &primitives[begin], 
                &primitives[end],
                [&](const Element &e) {
                    unsigned b = nBuckets * boundAll.offset(e.centroid())[axis];
                    if (b == nBuckets) b--;
                    return b <= minCostSplitBucket;
                }
            );
            newMid = pmid - &primitives[0];
            ret.childID[0] = recursiveConstruct(strategy, primitives, begin, newMid);
            ret.childID[1] = recursiveConstruct(strategy, primitives, newMid, end);
        }
    } // switch (strategy.algo)

    return retID;
}


template <class Element>
class Intersector<BVH<Element>> {
public:
    typedef typename Element::Intersection Intersection;

    /**
     *  @param r: ray for which intersections are checked
     */
    void intersect(
        const Ray& r,
        const BVH<Element>& tree,
        Intersection& res
    ) const {
        traverse(r, tree, res, tree.root());
    }
private:
    void traverse(
        const Ray& r,
        const BVH<Element>& tree,
        Intersection& res,
        typename BVH<Element>::NodeIterator it
    ) const {
        float tmin, tmax;
        if (!it->boundingBox().intersect(r.orig, r.dir, tmin, tmax)) {
            return;
        }
        if (it->isLeaf()) {
            // intersect ray wih all children
            auto elementIntersector = getIntersector<Element>();
            auto eit = it.elementsBegin();
            while (eit != it.elementsEnd()) {
                elementIntersector.intersect(r, *eit, res);
                ++eit;
            }
        } else {
            traverse(r, tree, res, it.leftChild());
            traverse(r, tree, res, it.rightChild());
        }
    }
};

}
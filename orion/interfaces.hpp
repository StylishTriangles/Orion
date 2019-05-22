/**
 * This header file contains definitions of some of the interfaces used in generic classes.
 * This interfaces are used to give a hint what templated classes require from provided type.
 **/

#pragma once

#include <typeinfo>

#include <orion/AABB.hpp>

namespace orion {

class Primitive {
public:
    // @returns the minimum of primitive's bounding box
    virtual vec3f lowerBound() const = 0;
    // @returns the maximum of primitive's bounding box
    virtual vec3f upperBound() const = 0;
    // @returns Axis Aligned Bounding Box
    AABB boundingBox() const {
        return AABB(this->lowerBound(), this->upperBound());
    }
    // @returns the central point of primitive's bounding box
    vec3f centroid() const {
        return this->boundingBox().centroid();
    }
};

template <class IntersectedType>
class Intersector {
    template <unsigned size>
    class Intersection {};
};

/** 
 *  @tparam IntersectedType: type which is bein intersected
 *  @returns intersector for provided class
 */
template <typename IntersectedType>
constexpr Intersector<IntersectedType> getIntersector(const IntersectedType&) {
    return Intersector<IntersectedType>();
}

/** 
 *  @tparam IntersectedType: type which is bein intersected
 *  @returns intersector for provided class
 */
template <typename IntersectedType>
constexpr Intersector<IntersectedType> getIntersector() {
    return Intersector<IntersectedType>();
}


} // namespace orion
/**
 *  If you wish to use AABB class, please #include geometry.hpp instead
 */
#pragma once

#include <orion/math.hpp>

namespace orion {

// AABB is an axis-aligned bounding box
class AABB {
public:
    AABB() : lo(F_INFINITY), hi(-F_INFINITY) {}
    AABB(const vec3f& lowerBound, const vec3f& upperBound) : 
        lo(lowerBound), hi(upperBound) {}

    // Calculate union of two bounding boxes
    // Create Bounding box that contains both bounding boxes
    AABB(const AABB& a, const AABB& b) :
        lo(min(a.lowerBound(), b.lowerBound())), 
        hi(max(a.upperBound(), b.upperBound())) 
    {}

    // Create Bounding box that contains both bounding box and the point
    AABB(const AABB& a, const vec3f& point) :
        lo(min(a.lowerBound(), point)), 
        hi(max(a.upperBound(), point)) 
    {}

    float surfaceArea() const {
        if (lo.x() > hi.x() && lo.y() > hi.y() && lo.z() > hi.z()) // empty AABB
            return 0;
        float dX = hi.x() - lo.x();
        float dY = hi.y() - lo.y();
        float dZ = hi.z() - lo.z();

        float front = dX*dY;
        float bottom = dX*dZ;
        float side = dY*dZ;

        return 2 * (front + bottom + side);
    }

    vec3f lowerBound() const { return lo; }
    vec3f upperBound() const { return hi; }

    // @returns the dimension in which bounding volume extends the most
    int maximumExtent() const {
        float dX = hi.x() - lo.x();
        float dY = hi.y() - lo.y();
        float dZ = hi.z() - lo.z();

        if (dX > dY && dX > dZ)
            return 0;
        else if (dY > dZ)
            return 1;
        else
            return 2;
    }

    // @returns the continuous position of a point relative to the corners of the box,
    // where a point at the minimum corner has offset [0,0,0], a point at the maximum corner has offset [1,1,1], and so forth.
    vec3f offset(vec3f point) const {
        vec3f o = point - lo;
        o = o / (hi - lo);
        return o;
    }

    // @returns central point of the obunding box
    vec3f centroid() const {
        return vec3f(0.5f) * (lo + hi);
    }

    /**
     * @param orig (in):  point of ray's origin
     * @param dir (in):   direction vector
     * @param t (out):    distance R(t) = O + tD 
     **/
    bool intersect(const vec3f &orig, 
                   const vec3f &dir,
                   float &tmin,
                   float &tmax
    ) const {
        vec3f inv_dir = vec3f(1.0f)/dir;

        vec3f t1 = (lo - orig)*inv_dir;
        vec3f t2 = (hi - orig)*inv_dir;

        vec3f tlower = min(t1, t2);
        vec3f thigher = max(t1, t2);

        tmin = max(tlower.x(), max(tlower.y(), tlower.z()));
        tmax = min(thigher.x(), min(thigher.y(), thigher.z()));

        if ((tmax > tmin) && (tmax > 0.0f)) {
            return true;
        }
        return false;
    }

private:
    // lo is the lower bound
    vec3f lo;
    // hi is the higher bound
    vec3f hi;
}; // class AABB

} // namespace orion
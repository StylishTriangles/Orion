#ifndef ORION_GEOMETRY_HPP
#define ORION_GEOMETRY_HPP

#include <cmath>

#include <orion/math.hpp>

// #include "avx/packed_geometry.hpp"

namespace orion {

class Sphere
{
public:
    Sphere();
    ~Sphere();

    // center of the sphere
    vec3f center;
    // color of surface and emission color of light
    vec3f color, emissionColor;
    // surface transparency and reflectivity
    float transparency, reflection;
    // radius and radius squared
    float radius, radius2;

    bool intersect(const vec3f &rayorig, const vec3f &raydir, float &t0, float &t1) const
    {
        vec3f l = center - rayorig;
        float tca = dot(l, raydir);
        if (tca < 0) 
            return false;
        float d2 = dot(l, l) - tca * tca;
        if (d2 > radius2) 
            return false;
        float thc = sqrt(radius2 - d2);
        t0 = tca - thc;
        t1 = tca + thc;

        return true;
    } 
};

// Triangle class to satisfy all mesh needs.
// Should be aligned to 16B.
class Triangle {
public:
    // Triangle members
    vec3f v0, e1, e2;

    Triangle() = default;

    Triangle(vec3f v0, vec3f v1, vec3f v2) :
        v0(v0), e1(v1-v0), e2(v2-v0) {}

    /**
     * Tomas Moller and Ben Trumbore method.
     * @param orig (in):      point of ray's origin
     * @param dir (in):       direction vector
     * @param t_out (in/out): distance R(t) = O + tD (is updated if this intersection is closer)
     * @param u_out (out):    distance to intersection from v0 via vector v1-v0
     * @param v_out (out):    distance to intersection from v0 via vector v2-v0
     * parameters u and v could be used for texture UV mapping.
     **/
    bool intersect(const vec3f &orig, 
                   const vec3f &dir,
                   float &t_out, 
                   float &u_out, 
                   float &v_out
    ) const {
        const float eps = 0.000001f;
        vec3f tvec, pvec, qvec;
        float det, inv_det;
        float t, u, v;

        // Find vectors for 2 edges sharing v0
        // e1 = v1 - v0; // Removed due to preprocessing of these values
        // e2 = v2 - v0;

        // begin calculating determinant - also used to create u parameter
        pvec = cross(dir, e2);

        // If determinant is near 0, ray lies in plane of triangle
        det = dot(e1, pvec);

        /*** we skip the culling branch so that we trace 2-sided triangles ***/

        if (det > -eps && det < eps)
            return 0;
        inv_det = 1.0f/det;

        // Calculate distance from vert0 to ray origin
        tvec = orig - v0;

        // Calculate U parameter and test bounds
        u = dot(tvec, pvec) * inv_det;
        if (u < 0 || u > 1)
            return 0;

        // Prepare to test v parameter
        qvec = cross(tvec, e1);

        // Calculate v parameter and test bounds
        v = dot(dir, qvec) * inv_det;
        if (v < 0 || u + v > 1)
            return 0;
        
        // Calculate t, ray intersects triangle
        t = dot(e2, qvec) * inv_det;

        // Check if intersection doesn't improve current result or it's behind us
        if (t > t_out || t < 0)
            return 0;
        
        // update u and v
        u_out = u;
        v_out = v;
        t_out = t;

        return 1;
    }

    // @returns normal to the surface of triangle
    vec3f normal() const {
        return cross(e1, e2);
    }
};

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

}; // namespace orion

#endif // ORION_GEOMETRY_HPP
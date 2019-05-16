#ifndef ORION_GEOMETRY_HPP
#define ORION_GEOMETRY_HPP

#include <cmath>

#include <orion/AABB.hpp>
#include <orion/interfaces.hpp>
#include <orion/math.hpp>

// #include "avx/packed_geometry.hpp"

namespace orion {

class Ray {
public:
    Ray(vec3f orig, vec3f dir) :
        orig(orig),
        dir(dir)
    {}
    // Point of ray's origin
    vec3f orig;
    // Ray's direction vector
    vec3f dir;
};

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
class Triangle : public Primitive {
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

    vec3f lowerBound() const {
        vec3f v1 = v0+e1;
        vec3f v2 = v0+e2;
        return min(min(v0,v1), v2);
    }

    vec3f upperBound() const {
        vec3f v1 = v0+e1;
        vec3f v2 = v0+e2;
        return max(max(v0,v1), v2);
    }
}; // class Triangle

template<>
class Intersector<Triangle> {
public:
    class Intersection {
    public:
        Intersection() :
            t(F_INFINITY)
        {}
        // t parameter of intersection
        float t;
        // distance along edges
        float u;
        float v;

        // Returns true if this 
        bool intersected() {
            return t != F_INFINITY;
        }
    };

    void intersect(
        const Ray& r,
        const Triangle& t,
        Intersection &res
    ) const;
}; // class intersector

inline void Intersector<Triangle>::intersect(
        const Ray& r,
        const Triangle& tri,
        Intersection &res
) const {
    const float eps = 0.000001f;
    vec3f tvec, pvec, qvec;
    float det, inv_det;
    float t, u, v;

    // Find vectors for 2 edges sharing v0
    // e1 = v1 - v0; // Removed due to preprocessing of these values
    // e2 = v2 - v0;

    // begin calculating determinant - also used to create u parameter
    pvec = cross(r.dir, tri.e2);

    // If determinant is near 0, ray lies in plane of triangle
    det = dot(tri.e1, pvec);

    /*** we skip the culling branch so that we trace 2-sided triangles ***/

    if (det > -eps && det < eps)
        return;
    inv_det = 1.0f/det;

    // Calculate distance from vert0 to ray origin
    tvec = r.orig - tri.v0;

    // Calculate U parameter and test bounds
    u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1)
        return;

    // Prepare to test v parameter
    qvec = cross(tvec, tri.e1);

    // Calculate v parameter and test bounds
    v = dot(r.dir, qvec) * inv_det;
    if (v < 0 || u + v > 1)
        return;
    
    // Calculate t, ray intersects triangle
    t = dot(tri.e2, qvec) * inv_det;

    // Check if intersection doesn't improve current result or it's behind us
    if (t > res.t || t < 0)
        return;
    
    // update u and v
    res.u = u;
    res.v = v;
    res.t = t;
}


}; // namespace orion

#endif // ORION_GEOMETRY_HPP
#ifndef ORION_GEOMETRY_HPP
#define ORION_GEOMETRY_HPP

#include <cmath>

#include <orion/material.hpp>
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
     * @param orig (in):  point of ray's origin
     * @param dir (in):   direction vector
     * @param t (out):    distance R(t) = O + tD 
     * @param u (out):    distance to intersection from v0 via vector v1-v0
     * @param v (out):    distance to intersection from v0 via vector v2-v0
     * parameters u and v could be used for texture UV mapping.
     **/
    bool intersect(const vec3f &orig, 
                   const vec3f &dir,
                   float &t, 
                   float &u, 
                   float &v
    ) const {
        const float eps = 0.000001f;
        vec3f tvec, pvec, qvec;
        float det, inv_det;

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

        return 1;
    }

    // @returns normal to the surface of triangle
    vec3f normal() const {
        return cross(e1, e2);
    }
};

};

#endif // ORION_GEOMETRY_HPP
#ifndef ORION_PACKED_GEOMETRY
#define ORION_PACKED_GEOMETRY

#include <cassert>

#include <orion/geometry.hpp>
#include <orion/math.hpp>
#include <orion/avx/math.hpp>

namespace orion {

struct PackedRay {
    // construct packed ray from simple single ray
    PackedRay(vec3f std_origin, vec3f std_direction) {
        origin[0].fill(std_origin.x());
        origin[1].fill(std_origin.y());
        origin[2].fill(std_origin.z());

        direction[0].fill(std_direction.x());
        direction[1].fill(std_direction.y());
        direction[2].fill(std_direction.z());
    }

    // information about a single ray in SOA format
    vec8f origin[3];
    vec8f direction[3];
};

class PackedTriangles {
public:
    // vertex and 2 edges in SOA format
    vec8f v0[3], e1[3], e2[3];

    PackedTriangles() = default;

    PackedTriangles(const Triangle* tArray, int count) { fromTriangles(tArray, count); }

    // @brief Create packed triangle from array of triangles
    // @param tArray: pointer to triangle array
    // @param count: how many triangles to fetch [1-8]
    void fromTriangles(const Triangle* tArray, int count) {
        assert (count >= 1 && count <= 8);
        alignas(32) float v0_raw[3*8];
        alignas(32) float e1_raw[3*8];
        alignas(32) float e2_raw[3*8];
        for (int i = 0; i < 8; i++) {
            const Triangle* curr;
            if (i >= count) {
                // in case we exceeded array size we will duplicate last available triangle
                curr = tArray + (count-1);
            } else {
                curr = tArray + i ;
            }

            v0_raw[ 0+i] = curr->v0.x();
            v0_raw[ 8+i] = curr->v0.y();
            v0_raw[16+i] = curr->v0.z();

            e1_raw[ 0+i] = curr->e1.x();
            e1_raw[ 8+i] = curr->e1.y();
            e1_raw[16+i] = curr->e1.z();

            e2_raw[ 0+i] = curr->e2.x();
            e2_raw[ 8+i] = curr->e2.y();
            e2_raw[16+i] = curr->e2.z();
        }
        for (int i = 0; i < 3; i++) {
            v0[i].load_aligned(v0_raw+(i*8));
            e1[i].load_aligned(e1_raw+(i*8));
            e2[i].load_aligned(e2_raw+(i*8));
        }
    }

    /**
     * Tomas Moller and Ben Trumbore method expanded to utilize AVX.
     * @param ray:        ray to test intersections with
     * @param t (in/out): distance R(t) = O + tD (updates iff lower t is found)
     * @param u (out):    distance to intersection from v0 via vector v1-v0 (updates iff t updates)
     * @param v (out):    distance to intersection from v0 via vector v2-v0 (updates iff t updates)
     * @returns int: index of intersected triangle (-1 on failure)
     * parameters u and v could be used for texture UV mapping.
     **/
    int intersect(const PackedRay &ray,
                  float &t, 
                  float &u, 
                  float &v) const 
    {
        const vec8f zero = vec8f(0.0f);
        const vec8f one = vec8f(1.0f);
        const vec8f plusEps = vec8f(0.000001f);
        const vec8f minusEps = vec8f(-0.000001f);
        const vec8f plusInf = vec8f(F_INFINITY);
        vec8f tvec[3], pvec[3], qvec[3];
        vec8f det, inv_det;

        // begin calculating determinant - also used to create u parameter
        multi_cross(pvec, ray.direction, e2);
        
        // If determinant is near 0, ray lies in plane of triangle
        det = multi_dot(e1, pvec);

        inv_det = one / det; // may overflow, all checks are performed at the end

        // Calculate distance from vert0 to ray origin
        multi_sub(tvec, ray.origin, v0);

        // Calculate U parameters
        vec8f us = multi_dot(tvec, pvec) * inv_det;

        multi_cross(qvec, tvec, e1);

        // Calculate V parameters
        vec8f vs = multi_dot(ray.direction, qvec) * inv_det;

        // Calculate t parameters
        vec8f ts = multi_dot(e2, qvec) * inv_det;

        // Compute which intersections have failed.
        // All operations are performed in the original order.
        vec8f failed = ((det > minusEps) & (det < plusEps)) // determinant near 0 - ray lies in plane of triangle
                     | (us < zero)
                     | (vs < zero)
                     | ((us + vs) > one);

        vec8f results = blendv(ts, plusInf, failed);

        // find minimal t (less than the current t)
        int index = -1;
        for (int i = 0; i < 8; i++) {
            if (results[i] < t) {
                index = i;
                t = results[i];
            }
        }
        if (index != -1) {
            u = us[index];
            v = vs[index];
        }
        
        return index;
    }

    // Count is the number of triangles packed present in PackedTriangles structure
    static const unsigned count = 8;

private:
    // Calculate cross product of vectors in SoA format
    void multi_cross(vec8f res[3], const vec8f a[3], const vec8f b[3]) const {
        // Let the compiler put FMA instructions here for us
        res[0] = a[1] * b[2] - a[2] * b[1];
        res[1] = a[2] * b[0] - a[0] * b[2];
        res[2] = a[0] * b[1] - a[1] * b[0];
    }

    // Calculate dot product of vectors in SoA format
    vec8f multi_dot(const vec8f a[3], const vec8f b[3]) const {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    // Substract each of b vectors from a vectors and store result in res
    void multi_sub(vec8f res[3], const vec8f a[3], const vec8f b[3]) const {
        res[0] = a[0] - b[0];
        res[1] = a[1] - b[1];
        res[2] = a[2] - b[2];
    }
};

}; // namespace orion

#endif // ORION_PACKED_GEOMETRY
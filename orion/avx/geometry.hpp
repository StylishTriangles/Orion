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

        vec3f inv_dir = vec3f(1.0f)/std_direction;
        inv_direction[0].fill(inv_dir.x());
        inv_direction[1].fill(inv_dir.y());
        inv_direction[2].fill(inv_dir.z());
    }

    // information about a single ray in SOA format
    vec8f origin[3];
    vec8f direction[3];
    vec8f inv_direction[3];
};

class PackedTriangles {
public:
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

        float mini;
        // Get the t with minimum index from results
        int min_index = min_in_vector_index(results, mini);
        if (mini < t) {
            t = mini;
            u = us[min_index];
            v = vs[min_index];
        }
        else {
            min_index = -1;
        }
        return min_index;
    }

    float minX() const {
        return minAxis(0);
    }
    float minY() const {
        return minAxis(1);
    }
    float minZ() const {
        return minAxis(2);
    }

    float maxX() const {
        return maxAxis(0);
    }
    float maxY() const {
        return maxAxis(1);
    }
    float maxZ() const {
        return maxAxis(2);
    }

    // Count is the number of triangles packed present in PackedTriangles structure
    static const unsigned count = 8;

private:
    // vertex and 2 edges in SOA format
    vec8f v0[3], e1[3], e2[3];

    float minAxis(int axisID) const {
        vec8f v0_ax = v0[axisID];
        vec8f v1_ax = v0[axisID] + e1[axisID];
        vec8f v2_ax = v0[axisID] + e2[axisID];

        float v0_min = min_in_vector(v0_ax);
        float v1_min = min_in_vector(v1_ax);
        float v2_min = min_in_vector(v2_ax);

        return min(min(v0_min, v1_min), v2_min);
    }

    float maxAxis(int axisID) const {
        vec8f v0_ax = v0[axisID];
        vec8f v1_ax = v0[axisID] + e1[axisID];
        vec8f v2_ax = v0[axisID] + e2[axisID];

        float v0_max = max_in_vector(v0_ax);
        float v1_max = max_in_vector(v1_ax);
        float v2_max = max_in_vector(v2_ax);

        return max(max(v0_max, v1_max), v2_max);
    }
};

// AVX SOA representation of 8 Axis Aligned Bounding Boxes
class PackedAABB {
public:
    PackedAABB() = default;

    void fromPackedTriangles(const PackedTriangles* pArray, int count) {
        assert (count >= 1 && count <= 8);
        alignas(32) float lo_raw[3*8];
        alignas(32) float hi_raw[3*8];

        for (int i = 0; i < 8; i++) {
            const PackedTriangles* pCurr;
            // Just like previously if there are less than 8 triangles just fill the remainder with legit packed triangles
            if (i >= count) {
                pCurr = pArray + (count-1);
            } else {
                pCurr = pArray + i;
            }

            lo_raw[ 0+i] = pCurr->minX();
            lo_raw[ 8+i] = pCurr->minY();
            lo_raw[16+i] = pCurr->minZ();

            hi_raw[ 0+i] = pCurr->maxX();
            hi_raw[ 8+i] = pCurr->maxY();
            hi_raw[16+i] = pCurr->maxZ();

        }
        for (int i = 0; i < 3; i++) {
            lo[i].load_aligned(lo_raw+(i*8));
            hi[i].load_aligned(hi_raw+(i*8));
        }
    }

    // Create bounding box for bounding boxes
    void fromPackedAABB(const PackedAABB* pArray, int count) {
        assert (count >= 1 && count <= 8);
        alignas(32) float lo_raw[3*8];
        alignas(32) float hi_raw[3*8];

        for (int i = 0; i < 8; i++) {
            const PackedAABB* pCurr;
            // Just like previously if there are less than 8 AABB just fill the remainder with legit packed AABB
            if (i >= count) {
                pCurr = pArray + (count-1);
            } else {
                pCurr = pArray + i;
            }

            lo_raw[ 0+i] = pCurr->minX();
            lo_raw[ 8+i] = pCurr->minY();
            lo_raw[16+i] = pCurr->minZ();

            hi_raw[ 0+i] = pCurr->maxX();
            hi_raw[ 8+i] = pCurr->maxY();
            hi_raw[16+i] = pCurr->maxZ();

        }
        for (int i = 0; i < 3; i++) {
            lo[i].load_aligned(lo_raw+(i*8));
            hi[i].load_aligned(hi_raw+(i*8));
        }
    }

    /** @brief Intersect ray with AABB
     *  @param ray: Single packed ray
     *  @param intersectionMask (out): mask representing which bounding boxes were intersected 
     *  @param tmin: closer point of intersection
     *  @param tmax: further point of intersection
     *  note: TRUE = 0xFFFFFFFF, FALSE = 0x00000000
     **/
    void intersect(const PackedRay& ray, vec8i& intersectionMask, vec8f& tmin, vec8f& tmax) {
        const vec8f zero = vec8f(0.0f);

        vec8f t1[3];
        vec8f t2[3];
        vec8f tmp[3];
        // t1 = (lo - rayO) * invDir
        multi_sub(tmp, lo, ray.origin);
        multi_mult(t1, tmp, ray.inv_direction);
        // t2 = (hi - rayO) * invDir
        multi_sub(tmp, hi, ray.origin);
        multi_mult(t2, tmp, ray.inv_direction);

        // intersection happens when ray intersects AABB on ech axis
        // so each of t1 must be lower than each of t2
        tmin = max(t1[0], max(t1[1], t1[2]));
        tmax = min(t2[0], min(t2[1], t2[2]));

        vec8f mask = (tmax > tmin) & (tmax > zero);
        // cast mask to create intersection mask
        intersectionMask = vec8i(_mm256_castps_si256(mask.vec)); 
    }

    float minX() const {
        return minAxis(0);
    }
    float minY() const {
        return minAxis(1);
    }
    float minZ() const {
        return minAxis(2);
    }

    float maxX() const {
        return maxAxis(0);
    }
    float maxY() const {
        return maxAxis(1);
    }
    float maxZ() const {
        return maxAxis(2);
    }

    // Count is the number of Bounding boxes packed present in PackedAABB structure
    static const unsigned count = 8;

#ifndef RAW_ACCESS_TESTING
private:
#endif
    // lower corner satisfying lo[i] <= hi[i]
    vec8f lo[3];
    // upper corner satisfying hi[i] >= lo[i]
    vec8f hi[3];

    float minAxis(int axisID) const {
        return min_in_vector(lo[axisID]);
    }

    float maxAxis(int axisID) const {
        return max_in_vector(hi[axisID]);
    }
};

}; // namespace orion

#endif // ORION_PACKED_GEOMETRY
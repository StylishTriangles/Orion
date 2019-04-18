#ifndef ORION_PACKED_MATH
#define ORION_PACKED_MATH

#include <immintrin.h>

#include <orion/math.hpp>

namespace orion {

struct vec8f {

    // default constructor - no guarantees
    vec8f() = default;

    // store in reverse order so that vec[0] corresponds to f0
    vec8f(float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7) : vec(_mm256_set_ps(f7,f6,f5,f4,f3,f2,f1,f0)) {}

    vec8f(float xx) : vec(_mm256_set1_ps(xx)) {}

    vec8f(__m256 ymm) : vec(ymm) {}

    vec8f(const vec8f &) = default;

    ~vec8f() = default;

    /** vec8f access operations **/
    void load(float* data) { vec = _mm256_loadu_ps(data); }
    void load_aligned(float* data) { vec = _mm256_load_ps(data); }

    // vector will be filled with xx
    void fill(float xx) { vec = _mm256_set1_ps(xx); }

    // access operators
    float operator [] (int index) const {return vec[index];}

    // members
    __m256 vec;
};

/** vec8f relational operators 
 *  Each operator performs a comparison of elements in a and b
 *  Comparisons will not cause FPE exceptions and are "ordered"
 *  Meaning every comparison with NaN returns FALSE
 *  FALSE = 0x00000000
 *  TRUE  = 0xFFFFFFFF
 **/ 

static inline vec8f operator < (const vec8f &a, const vec8f &b) {
    return _mm256_cmp_ps(a.vec, b.vec, _CMP_LT_OQ);
}

static inline vec8f operator > (const vec8f &a, const vec8f &b) {
    return _mm256_cmp_ps(a.vec, b.vec, _CMP_GT_OQ);
}

/** vec8f math **/
static inline vec8f operator * (const vec8f &a, const vec8f &b) {
    return a.vec * b.vec;
}

static inline vec8f operator / (const vec8f &a, const vec8f &b) {
    return a.vec / b.vec;
}

static inline vec8f operator + (const vec8f &a, const vec8f &b) {
    return a.vec + b.vec;
}

static inline vec8f operator - (const vec8f &a, const vec8f &b) {
    return a.vec - b.vec;
}

/** vec8f binary operations **/
static inline vec8f operator | (const vec8f &a, const vec8f &b) {
    return _mm256_or_ps(a.vec, b.vec);
}

static inline vec8f operator & (const vec8f &a, const vec8f &b) {
    return _mm256_and_ps(a.vec, b.vec);
}

// Multiply packed floats in a and b add the intermediate result to c and return the sum
// return a * b + c
static inline vec8f fmadd(const vec8f &a, const vec8f &b, const vec8f &c) {
    return _mm256_fmadd_ps(a.vec, b.vec, c.vec);
}

// Multiply packed floats in a and b substract c from the intermediate result
// return a * b - c
static inline vec8f fmsub(const vec8f &a, const vec8f &b, const vec8f &c) {
    return _mm256_fmsub_ps(a.vec, b.vec, c.vec);
}

// Blend vector elements from a and b using mask
// use b[i] if mask[i] is true
// else use a[i]
static inline vec8f blendv(const vec8f &a, const vec8f &b, const vec8f &mask) {
    return _mm256_blendv_ps(a.vec, b.vec, mask.vec);
}

static inline float min_in_vector(const vec8f &a) {
    // TODO: Divide and conquer?
    float mini = F_INFINITY;
    for (int i = 0; i < 8; i++) {
        if (a[i] < mini) {
            mini = a[i];
        }
    }
    return mini;
}


}; // namespace orion

#endif //ORION_PACKED_MATH
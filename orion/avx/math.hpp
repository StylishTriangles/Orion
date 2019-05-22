#ifndef ORION_PACKED_MATH
#define ORION_PACKED_MATH

#include <immintrin.h>

#include <orion/math.hpp>

namespace orion {

template<unsigned size>
class Vec3f {
public:

union {
__m256 stuff[3];
};
};

template<unsigned size>
Vec3f<size> operator+ (const Vec3f<size>& lhs, const Vec3f<size>& rhs) {
    Vec3f<size> res;
    for (int i = 0; i < 3; i++) {
        res.stuff[i] = lhs.stuff[i] + rhs.stuff[i];
    }
    return res;
}

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
    void load_unaligned(float* data) { vec = _mm256_loadu_ps(data); }
    void load_aligned(float* data) { vec = _mm256_load_ps(data); }

    void store_unaligned(float* dest) { _mm256_storeu_ps(dest, vec); }
    void store_aligned(float* dest) { _mm256_store_ps(dest, vec); }

    // vector will be filled with xx
    void fill(float xx) { vec = _mm256_set1_ps(xx); }

    // access operators
    float operator [] (int index) const {return vec[index];}

    // members
    __m256 vec;
};

struct vec8i {
    vec8i() = default;

    explicit vec8i(__m256i ymm) : vec(ymm) {}

    // access operators
    int operator [] (int index) const {
        switch(index) {
        case 0: return _mm256_extract_epi32(vec, 0); 
        case 1: return _mm256_extract_epi32(vec, 1); 
        case 2: return _mm256_extract_epi32(vec, 2); 
        case 3: return _mm256_extract_epi32(vec, 3); 
        case 4: return _mm256_extract_epi32(vec, 4); 
        case 5: return _mm256_extract_epi32(vec, 5); 
        case 6: return _mm256_extract_epi32(vec, 6); 
        case 7: return _mm256_extract_epi32(vec, 7);
        default: return -1; 
        }
    }

    __m256i vec;
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

// specialization of min function
template <>
inline vec8f min<vec8f>(const vec8f& a, const vec8f& b) {
    return _mm256_min_ps(a.vec, b.vec);
}

// specialization of max function
template <>
inline vec8f max<vec8f>(const vec8f& a, const vec8f& b) {
    return _mm256_max_ps(a.vec, b.vec);
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

// Find the minimum value in vector
static inline float min_in_vector(const vec8f &a) {
    // Divide and Conquer!
    const __m256 &va = a.vec;
    __m256 va_perm = _mm256_permute_ps(va, _MM_SHUFFLE(2,3,0,1)); // swap in pairs 2<->3 and 0<->1
    __m256 t1      = _mm256_min_ps(va, va_perm); // note that t1[0] = t1[1] and t1[2] = t1[3] (considering each of 128 bit lanes)
    __m256 t1_perm = _mm256_permute_ps(t1, _MM_SHUFFLE(0,0,2,2)); // swap upper 64 bits with lower 64 bits of each 128-bit lane (knowing above property)
    __m256 t2      = _mm256_min_ps(t1, t1_perm); // now t2[0] = t2[1] = t2[2] = t2[3] (considering each of 128 bit lanes)
    __m256 t2_perm = _mm256_permute2f128_ps(t2, t2, 1); // this basically swaps lower 128 bits and higher 128 bits
    __m256 t3      = _mm256_min_ps(t2, t2_perm);
    return _mm256_cvtss_f32(t3);
}

// Find the index of the minimum value in vector
// warning: No NaNs should be in a
// @param a: vector in which mininimum is searched
// @param minVal (out): minimum value in the vector
// @returns index of the minimum value in vector a
static inline int min_in_vector_index(const vec8f &a, float& minVal) {
    // Divide and Conquer!
    const __m256 &va = a.vec;
    __m256 va_perm = _mm256_permute_ps(va, _MM_SHUFFLE(2,3,0,1)); // swap in pairs 2<->3 and 0<->1
    __m256 t1      = _mm256_min_ps(va, va_perm); // note that t1[0] = t1[1] and t1[2] = t1[3] (considering each of 128 bit lanes)
    __m256 t1_perm = _mm256_permute_ps(t1, _MM_SHUFFLE(0,0,2,2)); // swap upper 64 bits with lower 64 bits of each 128-bit lane (knowing above property)
    __m256 t2      = _mm256_min_ps(t1, t1_perm); // now t2[0] = t2[1] = t2[2] = t2[3] (considering each of 128 bit lanes)
    __m256 t2_perm = _mm256_permute2f128_ps(t2, t2, 1); // this basically swaps lower 128 bits and higher 128 bits
    __m256 t3      = _mm256_min_ps(t2, t2_perm);
    // Until this point same as min_in_vector //
    __m256 mask  = _mm256_cmp_ps(va, t3, _CMP_EQ_OQ);
    int    indx  = _tzcnt_u32(_mm256_movemask_ps(mask));
    minVal = _mm256_cvtss_f32(t3);
    
    return indx;
}

static inline float max_in_vector(const vec8f &a) {
    // Divide and Conquer!
    const __m256 &va = a.vec;
    __m256 va_perm = _mm256_permute_ps(va, _MM_SHUFFLE(2,3,0,1)); // swap in pairs 2<->3 and 0<->1
    __m256 t1      = _mm256_max_ps(va, va_perm); // note that t1[0] = t1[1] and t1[2] = t1[3] (considering each of 128 bit lanes)
    __m256 t1_perm = _mm256_permute_ps(t1, _MM_SHUFFLE(0,0,2,2)); // swap upper 64 bits with lower 64 bits of each 128-bit lane (knowing above property)
    __m256 t2      = _mm256_max_ps(t1, t1_perm); // now t2[0] = t2[1] = t2[2] = t2[3] (considering each of 128 bit lanes)
    __m256 t2_perm = _mm256_permute2f128_ps(t2, t2, 1); // this basically swaps lower 128 bits and higher 128 bits
    __m256 t3      = _mm256_max_ps(t2, t2_perm);
    return _mm256_cvtss_f32(t3);
}

/** Non-canon math functions **/

// Calculate cross product of vectors in SoA format
static inline void multi_cross(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    // Let the compiler put FMA instructions here for us
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}

// Calculate dot product of vectors in SoA format
static inline vec8f multi_dot(const vec8f a[3], const vec8f b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Substract each of b vectors from a vectors and store result in res
static inline void multi_sub(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = a[0] - b[0];
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
}

// For each element: res[i] = a[i] + b[i]
static inline void multi_add(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = a[0] + b[0];
    res[1] = a[1] + b[1];
    res[2] = a[2] + b[2];
}

// For each element: res[i] = a[i] / b[i]
static inline void multi_div(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = a[0] / b[0];
    res[1] = a[1] / b[1];
    res[2] = a[2] / b[2];
}

// For each element: res[i] = a[i] * b[i]
static inline void multi_mult(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = a[0] * b[0];
    res[1] = a[1] * b[1];
    res[2] = a[2] * b[2];
}

// For each element: res[i] = min(a[i], b[i])
static inline void multi_min(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = min(a[0], b[0]);
    res[1] = min(a[1], b[1]);
    res[2] = min(a[2], b[2]);
}

// For each element: res[i] = max(a[i], b[i])
static inline void multi_max(vec8f res[3], const vec8f a[3], const vec8f b[3]) {
    res[0] = max(a[0], b[0]);
    res[1] = max(a[1], b[1]);
    res[2] = max(a[2], b[2]);
}

}; // namespace orion

#endif //ORION_PACKED_MATH
/*  Hello welcome to my math library, it's probably just vector math but
 *  the cool thing is that this stuff uses SIMD as much as possible.
 *  The sad fact is that this library may not work on big endian systems and old CPUs.
 */

#ifndef ORION_MATH_HPP
#define ORION_MATH_HPP

#include <immintrin.h>

namespace orion {

struct alignas(16) vec4f 
{
    // vec4f constructors

    // default constructor - no guarantees
    vec4f() = default;

    // Floats are stored in reverse order because it's how SSE works
    vec4f(float x, float y, float z, float w) { vec = _mm_set_ps(w,z,y,x); }

    vec4f(float xx) { vec = _mm_set_ps1(xx);}

    vec4f(__m128 xmm) : vec(xmm) {}

    vec4f(const vec4f & origin) = default;

    ~vec4f() = default;

    // vec4f access operators

    float& operator [] (int index) {return vec[index];}
    const float& operator [] (int index) const {return vec[index];}

    float& x() {return vec[0];}
    const float& x() const {return vec[0];}

    float& y() {return vec[1];}
    const float& y() const {return vec[1];}

    float& z() {return vec[2];}
    const float& z() const {return vec[2];}

    float& w() {return vec[3];}
    const float& w() const {return vec[3];}

    // vec4f arithmetic operators

    vec4f& operator *= (vec4f m) {vec *= m.vec; return *this;}

    // vec4f Math functions

    // dot - compute dot product of two vectors
    float dot(vec4f a, vec4f b) const {
        // constant explanation:
        // 0xf1 == 0b11110001
        // Upper four bits are used to determine which elements of xmm vector will be used in the dot product
        // Lower four bits determine which elements of returned xmm vector will contain the result
        return _mm_cvtss_f32(_mm_dp_ps(a.vec, b.vec, 0xf1));
    }

    void normalize() {
		__m128 dp = _mm_dp_ps(vec, vec, 0xFF); 
		dp = _mm_rsqrt_ps(dp);
		vec = _mm_mul_ps(vec, dp);
    }

    vec4f normalized() const {
		__m128 dp = _mm_dp_ps(vec, vec, 0xFF); 
		dp = _mm_rsqrt_ps(dp);
		return _mm_mul_ps(vec, dp);
    }
    
    // vec4f member

    // 128 bit SIMD vector
    __m128 vec;
};

struct alignas(16) vec3f : public vec4f {
    // vec3f constructors

    vec3f() = default;

    vec3f(float x, float y, float z) : vec4f(x,y,z,0) {}

    vec3f(float xx) : vec4f(xx) {}

    vec3f(__m128 xmm) : vec4f(xmm) {}

    vec3f(const vec3f & origin) = default;

    ~vec3f() = default;


    // vec3f arithmetic operators

    vec3f& operator *= (vec3f m) {vec *= m.vec; return *this;}
    vec3f& operator /= (vec3f m) {vec /= m.vec; return *this;}

    // vec3f access operators

    float& w() = delete;
    const float& w() const = delete;

    /** vec3f math functions **/

    // cross product of two 3-element vectors.
    // Below formula is explained here: 
    // http://threadlocalmutex.com/?p=8
    vec3f cross(vec3f vb) const {
        __m128 const &a = vec;
        __m128 const &b = vb.vec;

        __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
        __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
        __m128 c = _mm_sub_ps(_mm_mul_ps(a, b_yzx), _mm_mul_ps(a_yzx, b));
        return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
    }

    float dot(vec3f b) const {
        // constant explanation:
        // 0x71 == 0b01110001
        // Upper four bits are used to determine which elements of xmm vector will be used in the dot product
        // Lower four bits determine which elements of returned xmm vector will contain the result
        return _mm_cvtss_f32(_mm_dp_ps(vec, b.vec, 0x71));
    }

    void normalize() {
        __m128 dp = _mm_dp_ps(vec, vec, 0x7F); 
        dp = _mm_rsqrt_ps(dp);
        vec = _mm_mul_ps(vec, dp);
    }

    vec3f normalized() const {
        __m128 dp = _mm_dp_ps(vec, vec, 0x7F); 
        dp = _mm_rsqrt_ps(dp);
        return _mm_mul_ps(vec, dp);
    }

    // compute projection of vector v onto this.
    vec3f projectionOf(vec3f v) {
        __m128 n = _mm_dp_ps(vec, v.vec, 0x7f);
        __m128 d = _mm_dp_ps(vec,   vec, 0x7f);
        return (n/d)*vec;
    }
};

// Basic structure to hold 2 floats
// adapted to have similar API as vec4f
struct vec2f {
    vec2f() = default;

    vec2f(float x, float y) {vec[0] = x; vec[1] = y;}

    vec2f(float xx) { vec[0] = vec[1] = xx; }

    ~vec2f() = default;

    float& x() {return vec[0];}
    const float& x() const {return vec[0];}

    float& y() {return vec[1];}
    const float& y() const {return vec[1];}

    float vec[2];
};


// vec4f operators
vec4f operator * (vec4f lhs, vec4f rhs);

vec4f operator * (float x, vec4f v);

vec4f operator + (vec4f lhs, vec4f rhs);

vec4f operator + (vec4f lhs, float rhs);

vec4f operator + (float lhs, vec4f rhs);

vec4f operator - (vec4f lhs, vec4f rhs);

// vec3f operators
vec3f operator * (vec3f lhs, vec3f rhs);

vec3f operator * (float x, vec3f v);

vec3f operator + (vec3f lhs, vec3f rhs);

vec3f operator + (vec3f lhs, float rhs);

vec3f operator + (float lhs, vec3f rhs);

vec3f operator - (vec3f lhs, vec3f rhs);

}; // orion

#endif // ORION_MATH_HPP
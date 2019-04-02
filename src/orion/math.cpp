#include <orion/math.hpp>

namespace orion {

// vec4f operators
inline vec4f operator * (vec4f lhs, vec4f rhs) {
    return lhs.vec * rhs.vec;
}

inline vec4f operator * (float x, vec4f v) {
    return vec4f(x) * v;
}

inline vec4f operator + (vec4f lhs, vec4f rhs) {
    return lhs.vec + rhs.vec;
}

inline vec4f operator + (vec4f lhs, float rhs) {
    return lhs + vec4f(rhs);
}

inline vec4f operator + (float lhs, vec4f rhs) {
    return vec4f(lhs) + rhs;
}

inline vec4f operator - (vec4f lhs, vec4f rhs) {
    return lhs.vec + rhs.vec;
}

inline vec4f operator - (vec4f un) {
    const __m128 minusZero = _mm_set1_ps(-0.0f);
    return _mm_xor_ps(un.vec, minusZero);
}

// vec3f operators
vec3f operator * (vec3f lhs, vec3f rhs) {
    return lhs.vec * rhs.vec;
}

vec3f operator * (float x, vec3f v) {
    return vec3f(x) * v;
}

vec3f operator + (vec3f lhs, vec3f rhs) {
    return lhs.vec + rhs.vec;
}

vec3f operator + (vec3f lhs, float rhs) {
    return lhs + vec3f(rhs);
}

vec3f operator + (float lhs, vec3f rhs) {
    return vec3f(lhs) + rhs;
}

vec3f operator - (vec3f lhs, vec3f rhs) {
    return lhs.vec - rhs.vec;
}

vec3f operator - (vec3f un) {
    const __m128 minusZero = _mm_set1_ps(-0.0f);
    return _mm_xor_ps(un.vec, minusZero);
}

};
#pragma once

#include <orion/math.hpp>
#include <orion/avx/math.hpp>

namespace orion {

class Ray {
public:
    Ray(vec3f orig, vec3f dir) :
        orig(orig),
        dir(dir),
        inv_dir(1.0f/dir)
    {}
    // Point of ray's origin
    vec3f orig;
    // Ray's direction vector
    vec3f dir;
    // Inverse of ray's direction
    vec3f inv_dir;
};

template <unsigned size>
class PackedRay;

template <unsigned size>
class PackedRay {
public:
    // set ray at index
    void setRay(unsigned index, vec3f origin, vec3f direction) {
        vec3f invDir = vec3f(1.0f)/direction;
        setRay(index, origin, direction, invDir);
    }

    // set ray at index
    void setRay(unsigned index, vec3f origin, vec3f direction, vec3f inverse_direction) {
        mOrigin[index] = origin[0];
        mOrigin[index + size] = origin[1];
        mOrigin[index + 2*size] = origin[2];

        mDir[index] = direction[0];
        mDir[index + size] = direction[1];
        mDir[index + 2*size] = direction[2];

        mInvDir[index] = inverse_direction[0];
        mInvDir[index + size] = inverse_direction[1];
        mInvDir[index + 2*size] = inverse_direction[2];
    }

    void getOrigin(unsigned index, vec3f& origin) const {
        origin = vec3f(
            mOrigin[index],
            mOrigin[index+size],
            mOrigin[index+2*size]
        );
    }

    void getDirection(unsigned index, vec3f& dir) const {
        dir = vec3f(
            mDir[index],
            mDir[index+size],
            mDir[index+2*size]
        );
    }

    void getInvDirection(unsigned index, vec3f& inv_dir) const {
        inv_dir = vec3f(
            mInvDir[index],
            mInvDir[index+size],
            mInvDir[index+2*size]
        );
    }

private:
    // origin in SoA format
    float mOrigin[3*size];
    // direction in SoA format
    float mDir[3*size];
    // precomputed inv_dir
    float mInvDir[3*size];
}; // class PackedRay

// /**
//  * PackedRaySA is a ray representation in structure of arrays format
//  * It is intended for parallel operations on rays
//  */
// template <unsigned size>
// class PackedRaySA {
//     static_assert(size==8, "Only packed rays of size 8 are supported");
// };

// template <>
// class PackedRaySA<8> {
// public:
//     // create 8 rays, all with the same data as provided ray
//     PackedRaySA(const PackedRay<1> &ref) {
//         // clone and fill origin vector
//         orig[0] = vec8f(ref.orig[0]);
//         orig[1] = vec8f(ref.orig[1]);
//         orig[2] = vec8f(ref.orig[2]);
//         // clone and fill direction vector
//         dir[0] = vec8f(ref.dir[0]);
//         dir[1] = vec8f(ref.dir[1]);
//         dir[2] = vec8f(ref.dir[2]);
//         // clone and fill inverse direction vector
//         inv_dir[0] = vec8f(ref.inv_dir[0]);
//         inv_dir[1] = vec8f(ref.inv_dir[1]);
//         inv_dir[2] = vec8f(ref.inv_dir[2]);
//     }

//     // 
//     PackedRaySA(const PackedRay<8> &ref) {
//         alignas(32) float converter[3*8];
//         // load vector origins
//         for (unsigned i = 0; i < 8; i++) {
//             converter[0 +i] = ref[i].orig.x();
//             converter[8 +i] = ref[i].orig.y();
//             converter[16+i] = ref[i].orig.z();
//         }
//         orig[0].load_aligned(converter);
//         orig[1].load_aligned(converter+8);
//         orig[2].load_aligned(converter+16);
//         // load vector directions
//         for (unsigned i = 0; i < 8; i++) {
//             converter[0 +i] = ref[i].dir.x();
//             converter[8 +i] = ref[i].dir.y();
//             converter[16+i] = ref[i].dir.z();
//         }
//         dir[0].load_aligned(converter);
//         dir[1].load_aligned(converter+8);
//         dir[2].load_aligned(converter+16);
//         // load vector iverted directions
//         for (unsigned i = 0; i < 8; i++) {
//             converter[0 +i] = ref[i].inv_dir.x();
//             converter[8 +i] = ref[i].inv_dir.y();
//             converter[16+i] = ref[i].inv_dir.z();
//         }
//         inv_dir[0].load_aligned(converter);
//         inv_dir[1].load_aligned(converter+8);
//         inv_dir[2].load_aligned(converter+16);
//     }

//     // 8 vectors in SoA format
//     vec8f orig[3];
//     vec8f dir[3];
//     vec8f inv_dir[3];
// };

} // namespace orion
#pragma once

#include <cinttypes>
#include <type_traits>
#include <immintrin.h>

namespace orion {

template<unsigned size>
class Bitmask {
public:
    static_assert(size <= 32, "Maximum allowed size for bitmask is 32");

    void set(unsigned index, bool bValue) {
        unsigned value = bValue;
        mMask = (mMask & ~(1<<index)) | (value << index);
    }

    bool get(unsigned index) const {
        return (mMask)&(1<<index);
    }

    unsigned count() const {
        return _mm_popcnt_u32(mMask);
    }

private:
    uint32_t mMask = 0;
}; // class Bitmask


} // namespace orion
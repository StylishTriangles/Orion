#pragma once

#include <algorithm>
#include <cinttypes>

namespace orion {
    
template<class T>
class Array2D {
public:
    Array2D() : 
        mArray(nullptr) {}
    
    Array2D(uint64_t height, uint64_t width) {
        allocate(height, width);
    }

    Array2D(uint64_t height, uint64_t width, const T& intitialVal) {
        allocate(height, width);
        std::fill(mArray, mArray + width*height, intitialVal);
    }

    Array2D(const Array2D& ref)
    {
        allocate(ref.mHeight, ref.mWidth);
        std::copy(ref.mArray, ref.mArray + ref.size(), mArray);
    }

    ~Array2D() {
        cleanup();
    }

    // @returns pointer to row with index `height`
    T* operator[] (uint64_t h) {
        return mArray+h*mWidth;
    }

    const T* operator[] (uint64_t h) const {
        return mArray+h*mWidth;
    }

    // @returns row count
    uint64_t height() const {
        return mHeight;
    }

    // @returns column count
    uint64_t width() const {
        return mWidth;
    }

    // @returns total size of the structure
    uint64_t size() const {
        return mWidth*mHeight;
    }

    // @returns pointer to the first element (that at location [0][0])
    T* begin() {
        return mArray;
    }

    // @returns pointer to the first element (that at location [0][0])
    const T* begin() const {
        return mArray;
    }

    // @returns pointer to the last element (that at location [height-1][width-1])
    T* end() {
        return mArray + mWidth*mHeight;
    }

    // @returns pointer to the last element (that at location [height-1][width-1])
    const T* end() const {
        return mArray + mWidth*mHeight;
    }

private:
    void allocate(uint64_t height, uint64_t width) {
        mHeight = height;
        mWidth = width;
        mArray = new T[height*width];
    }
    void cleanup() { 
        delete[] mArray;
    }

    uint64_t mHeight;
    uint64_t mWidth;
    T *mArray;
}; // class Array2D

}; // namespace orion
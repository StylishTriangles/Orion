#pragma once

#include <orion/interfaces.hpp>
#include <orion/ray.hpp>
#include <orion/bvh.hpp>
#include <orion/avx/geometry.hpp>

/**
 *  @tparam Element: Contained type, must fulfill the Primitive interface
 */
template <class Element>
class SBVH : public Primitive {

};
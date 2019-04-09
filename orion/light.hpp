#ifndef ORION_LIGHT_H
#define ORION_LIGHT_H

#include <orion/math.hpp>

namespace orion {

class Light {
public:
    vec3f position;
    vec3f color;
    float intensity;
};

}

#endif // ORION_LIGHT_H
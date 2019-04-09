/**
 * This file file serves as an entry point in a standalone raytracer
 * Compile with -DRAYTRACER_STANDALONE to use this
 **/

#ifdef RAYTRACER_STANDALONE

#include <cstdio>
#include <orion/raytracer.hpp>

int main(int argc, char** argv)
{
    if (argc != 2) {
        printf("Usage:\nraytracer path/to/rtc/file.rtc\n");
        return 1;
    }
    orion::RayTracer RT;
    RT.traceRTC(argv[1]);
    return 0;
}

#endif
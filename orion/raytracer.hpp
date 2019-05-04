#ifndef ORION_RAYTRACER_HPP
#define ORION_RAYTRACER_HPP

#include <vector>

#include <orion/geometry.hpp>
#include <orion/light.hpp>
#include <orion/math.hpp>
#include <orion/model.hpp>
#include <orion/rtc_parser.hpp>

namespace orion {

class RayTracer;

// The entry point of Orion's raytracing.
// RayTracer class contains all scene information.
class RayTracer
{
public:
    RayTracer() = default;
    ~RayTracer() = default;

    // Simple function to trace a single model described in an rtc file 
    void traceRTC(const std::string& rtc_file_name, const std::string& path_to_image = "raytracer.png");

protected:
    void savePPM(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image);
    void savePNG(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image);

    vec3f trace(TracedModel &m, const vec3f &origin, const vec3f &dir, const int depth);

    /** 
     * @brief Calculate camera vectors based on loaded rtc.
     * @param rtcd: rtc data to be used
     * @param vecFront (out): vector pointing forward from the camera, represents screen center (it is also normalized)
     * @param vecUp (out): vector pointing upwards from the camera (scaled by vectical fov)
     * @param vecRight (out): vector perpendicular to vecFront and vecUp (scaled by fov and aspect ratio)
     * The top left corner of the screen can be described as front + up - right and bottom right as front - up + right.
     **/
    void calculateCameraVectors(const rtc_data &rtc, vec3f& vecFront, vec3f& vecUp, vec3f& vecRight);

    void printStatistics(TracedModel &m);

private:
    rtc_data rtc;
};

};

#endif // ORION_RAYTRACER_HPP
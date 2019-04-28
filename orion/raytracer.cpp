#include <filesystem>
#include <fstream>
#include <random>

#include <tqdm/tqdm.hpp>

#include <orion/raytracer.hpp>
#include <orion/random.hpp>


using namespace std;

namespace orion {

void RayTracer::traceRTC(const char* rtc_file_name, const char* path_to_image)
{
    filesystem::path rtc_path(rtc_file_name);
    // directory in which rtc file exists
    filesystem::path rtc_dir = rtc_path.parent_path();
    // parse the provided rtc file
    rtc = parse_rtc(rtc_file_name);

    // load obj files as marked by rtc
    filesystem::path obj_path = rtc_dir.append(rtc.obj_file);
    TracedModel m = TracedModel(obj_path.string().c_str());

    // define our image as vector of vectors of colors
    vector<vector<vec3f> > image(rtc.yres);
    for (int i = 0; i < rtc.yres; i++) {
        image[i].resize(rtc.xres);
        for (int j = 0; j < rtc.xres; j++) {
            image[i][j] = 0.0f;
        }
    }
    vec3f vecFront, vecUp, vecRight;
    calculateCameraVectors(rtc, vecFront, vecUp, vecRight);

    // Random ray direction will be chosen as: 
    // rf1 * vecRight + rf2 * vecUp,
    // where rfx is a random float in range (-1,1)
    uniform_real_distribution<float> randomFloat(-1,1);

    // Fast random number generator
    xoroshiro128 rng;

    auto progress = tq::trange(int(rtc.yres));
    progress.set_prefix("Raytracing ");
    for (int i: progress) {
        for (int j = 0; j < rtc.xres; j++) {
            float x = 2 * (float(j) / float(rtc.xres)) - 1;
            float y = 2 * (float(i) / float(rtc.yres)) - 1;
            y = -y; // flip y axis so that (-1, -1) is the top left corner
            vec3f dir = vecFront + x * vecRight + y * vecUp;
            // run raytracer on our model
            image[i][j] = trace(m, rtc.view_point, dir, 0);
        }
    }

    // save resulting image as ppm file
    savePPM(path_to_image, image);
}

vec3f RayTracer::trace(TracedModel &m, const vec3f &origin, const vec3f &dir, const int depth, bool shadow)
{
    vec3f color = 0.0f;
    if (shadow)
        color = 1.0f;
    // nearest intersection
    float tnear = F_INFINITY;
    
    MeshIntersection inter = m.intersect(origin, dir, tnear);
    // triangle not hit
    if (!inter.intersected())
        return color;
    
    // bias will be used to move our ray away from the surface on reflection
    const float bias = 1e-4;
    // calculate normal to surface
    vec3f normal = inter.normal().normalized();
    vec2f uvs = inter.texture_uv();
    // calculate point where ray hits the surface
    vec3f hitPos = origin + dir * tnear;

    for (Light const& lght: rtc.lights) {
        float tnear2 = F_INFINITY;
        MeshIntersection inter2 = m.intersect(hitPos+(bias*normal), lght.position-hitPos, tnear2);
        if (!inter2.intersected())
            color += inter.material().color(dir, normal, hitPos, lght, uvs);
    }

    return color;
}

void RayTracer::calculateCameraVectors(const rtc_data &rtcd, vec3f& vecFront, vec3f& vecUp, vec3f& vecRight)
{
    // copy vectors from loaded rtc structure
    vecUp = rtcd.vector_up;
    vecFront = rtcd.look_at - rtcd.view_point;

    // We want to use camera position as source for our rays.
    // The front vector will represent center of the screen.
    // The top and right vectors will point to edges of the screen.
    // So -1 * vecUp is the bottom edge and 1 * vecUp is the top one

    // Perform Gram-Schmidt orogonalization of 2 vectors
    vecUp = orthogonalize(vecFront, vecUp);

    // Normalize our vectors
    vecUp.normalize();
    vecFront.normalize();

    // Construct the right vector as the cross product of up and front.
    // A cross product of 2 unit vectors will also be unit.
    vecRight = cross(vecFront, vecUp);

    // Let's transform those vectors to reflect our screen better
    // so that we can use them  as ray directions.
    vecUp = vecUp * rtcd.y_view * 0.5f;
    vecRight = vecRight * rtcd.y_view * rtcd.aspect_ratio * 0.5f;
}

void RayTracer::savePPM(const char* path_to_image, const std::vector<std::vector<vec3f> > &image)
{
    // Save result to a PPM image (keep these flags if you compile under Windows)
    std::ofstream ofs(path_to_image, std::ios::out | std::ios::binary);
    ofs << "P6\n" << rtc.xres << " " << rtc.yres << "\n255\n";  // Use image width and height
    for (unsigned i = 0; i < unsigned(rtc.yres); i++) {
        for (unsigned j = 0; j < unsigned(rtc.xres); j++) {
            ofs << (unsigned char)(min(float(1), image[i][j].x()) * 255) <<
            (unsigned char)(min(float(1), image[i][j].y()) * 255) <<
            (unsigned char)(min(float(1), image[i][j].z()) * 255);
        }
    }
    ofs.close();
}

}; // namespace orion
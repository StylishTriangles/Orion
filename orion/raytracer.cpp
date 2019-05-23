#include <filesystem>
#include <fstream>
#include <random>

#include <tqdm/tqdm.hpp>
#include <stb_image_write.h>

#include <orion/array2d.hpp>
#include <orion/raytracer.hpp>
#include <orion/random.hpp>


using namespace std;

namespace orion {

void RayTracer::traceRTC(const std::string& rtc_file_name, const std::string& path_to_image)
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
    vector<vec3f> vZero(rtc.xres, vec3f(0.0f));
    vector<vector<vec3f> > image(rtc.yres, vZero);

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
        #pragma omp parallel for
        for (int j = 0; j < rtc.xres; j++) {
            float x = 2 * (float(j) / float(rtc.xres)) - 1;
            float y = 2 * (float(i) / float(rtc.yres)) - 1;
            y = -y; // flip y axis so that (-1, -1) is the top left corner
            vec3f dir = vecFront + x * vecRight + y * vecUp;
            // run raytracer on our model
            image[i][j] = trace(m, rtc.view_point, dir, rtc.recursion_level);
        }
    }
    std::cerr << "\n"; // newline after progress bar
    printStatistics(m);

    // save resulting image as ppm file
    savePNG(path_to_image, image);
}

vec3f RayTracer::trace(TracedModel &m, const vec3f &origin, const vec3f &dir, const int depth)
{
    vec3f color = 0.1f;
    // nearest intersection
    float tnear = F_INFINITY;
    
    MeshIntersection inter = m.intersect(origin, dir, tnear);
    // triangle not hit
    if (!inter.intersected())
        return color;
    
    // bias will be used to move our ray away from the surface on reflection
    // This value was chosen by trial and error - in a way that primary rays + shadow rays don't generate black pixels
    const float bias = 1e-3f;
    // calculate normal to surface
    vec3f normal = inter.normal().normalized();
    vec3f snormal = inter.surfaceNormal().normalized();
    // normal = snormal;
    vec2f uv = inter.texture_uv();
    // calculate point where ray hits the surface
    vec3f hitPos = origin + dir * tnear;

    for (Light const& lght: rtc.lights) {
        float tnear2 = F_INFINITY;
        MeshIntersection inter2 = m.intersect(hitPos+(bias*snormal), lght.position-hitPos, tnear2);
        if (!inter2.intersected())
            color += inter.material().color(dir, normal, hitPos, lght, uv);
    }

    // TODO: Tail recursion?
    if (depth > 0) {
        // TODO: recognize the edge case where normal is in the opposite direction from surfaceNormal
        color += inter.material().reflectivity(uv) * trace(m, hitPos + normal*bias, reflect(dir, normal), depth-1);
    }

    // color = (normal+1) / 2;

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

void RayTracer::savePPM(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image)
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

void RayTracer::savePNG(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image)
{
    struct RGB {
        unsigned char r,g,b,a;
    };
    unsigned height = rtc.yres;
    unsigned width = rtc.xres;
    Array2D<RGB> exporter(height, width);

    for (unsigned h = 0; h < height; h++) {
        for (unsigned w = 0; w < width; w++) {
            exporter[h][w].r = min(1.0f, image[h][w].x())*255;
            exporter[h][w].g = min(1.0f, image[h][w].y())*255;
            exporter[h][w].b = min(1.0f, image[h][w].z())*255;
            exporter[h][w].a = 255;
        }
    }

    int ret = stbi_write_png(path_to_image.c_str(), width, height, 4, exporter.begin(), width*sizeof(RGB));
    if (ret == 0) {
        std::cout << "Error ocurred when saving file to " << path_to_image << std::endl;
    }
}

void RayTracer::printStatistics(TracedModel &m)
{
    printf("Triangles:                  %d\n", m.triangleCount());
    // printf("Ray-AABB intersections:     %d\n", intersectionCount().first);
    // printf("Ray-Triangle intersections: %d\n", intersectionCount().second);
}

}; // namespace orion
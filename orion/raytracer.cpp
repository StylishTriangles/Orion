#include <filesystem>
#include <fstream>
#include <random>

#include <omp.h>

#include <tqdm/tqdm.hpp>
#include <stb_image_write.h>

#include <orion/array2d.hpp>
#include <orion/raytracer.hpp>
#include <orion/random.hpp>


using namespace std;

namespace orion {

void RayTracer::traceRTC(const std::string& rtc_file_name, const std::string& path_to_image, unsigned samples, unsigned light_samples, unsigned threads)
{
    this->light_samples = light_samples;
    if (threads == 0) {
        threads = omp_get_max_threads();
    }

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

    // Random number generator used to initialize threaded rngs
    xoroshiro128 init_rng;
    init_rng.seed((unsigned long)this);
    xoroshiro128 rng[threads];
    rng[0] = init_rng;
    for (unsigned i = 1; i < threads; i++) {
        rng[i] = rng[i-1];
        rng[i].jump();
    }

    // create a pixel fill pattern
    uniform_real_distribution<float> randomFloat(0,1);
    float pixelX = 2.0f/float(rtc.xres); // pixel width
    float pixelY = 2.0f/float(rtc.yres); // pixel heigh
    vector<pair<float,float>> pattern;
    for (unsigned i = 0; i < samples; i++) {
        pattern.push_back({
            pixelX*randomFloat(init_rng),
            pixelY*randomFloat(init_rng)
        });
    }
    float inv_samples = 1.0f/samples;

    auto progress = tq::trange(int(rtc.yres));
    this->max_depth = rtc.recursion_level;
    progress.set_prefix("Raytracing ");
    for (int i: progress) {
        #pragma omp parallel for num_threads(threads)
        for (int j = 0; j < rtc.xres; j++) {
            float x = 2 * (float(j) / float(rtc.xres)) - 1;
            float y = 2 * (float(i) / float(rtc.yres)) - 1;

            // sample pixels
            for (unsigned k = 0; k < samples; k++) { 
                float xsample = x + pattern[k].first;
                float ysample = y + pattern[k].second;
                ysample = -ysample; // flip y axis so that (-1, -1) is the top left corner
                vec3f dir = vecFront + (xsample * vecRight) + (ysample * vecUp);
                
                // run raytracer on our model
                vec3f tracedColor = trace(m, rng[omp_get_thread_num()], rtc.view_point, dir);
                image[i][j] += tracedColor;
            }
            image[i][j] *= inv_samples;
        }
    }
    std::cerr << "\n"; // newline after progress bar
    printStatistics(m);

    auto extension = filesystem::path(path_to_image).extension().string();
    if (extension == ".ppm") {
        savePPM(path_to_image, image);
    } else if (extension == ".png") {
        savePNG(path_to_image, image);
    } else if (extension == ".hdr") {
        saveHDR(path_to_image, image);
    } else {
        printf("Extension not recognized \"%s\" (will be saved as .hdr), supported extensions: .hdr .png .ppm\n", extension.c_str());
        saveHDR(path_to_image, image);
    }
}

vec3f RayTracer::trace(TracedModel &m, xoroshiro128& rng, const vec3f &origin, const vec3f &dir, const unsigned depth)
{
    vec3f color = 0.0f;
    // nearest intersection
    float tnear = F_INFINITY;
    
    MeshIntersection inter = m.intersect(origin, dir, tnear);
    // triangle not hit
    if (!inter.intersected())
        return 0.0f;
    
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

    if (depth == 0) // primary ray hits emissive surface
        color = inter.material().emissivity(uv) * inter.pMesh->surfaceArea() * dot(normalize(dir), -normal);

    // Assume BRDF if no lights present
    if (rtc.lights.empty()) {
        vec3f direct_light(0.0f);
        for (auto const& rMesh: m.emissiveSurfaces()) {
            // sample light source
            vec3f direct_contribution(0.0f);
            for (unsigned i = 0; i < light_samples; i++) {
                float tnear2 = F_INFINITY;
                // Use Monte Carlo methods to determine if light is visible
                vec3f target;
                float light_bias;
                rMesh.randomPointOnSurface(rng, target, light_bias);

                MeshIntersection inter2 = m.intersect(
                    hitPos+(bias*snormal), 
                    target-hitPos, 
                    tnear2
                );
                
                // Ray has to intersect with exactly the light source, therefore when t = 1
                if (inter2.intersected() && inter2.pMesh->mID == rMesh.mID) { // ugly hack to check if we intersected with intended light src
                    Light l;
                    l.position = target;
                    l.intensity = light_bias * rMesh.surfaceArea();
                    l.color = rMesh.material().emissivity(inter2.texture_uv());
                    direct_contribution += inter.material().colorBRDF(normal, hitPos, l, inter2.normal().normalized(), uv);
                }
            }
            color += 1.0f/light_samples * direct_contribution;
        }
        // direct light calculation finished, return if there is no depth remaining
        if (depth >= max_depth)
            return color + direct_light;
        
        // russian roulette
        vec3f kd = inter.material().diffuse(uv);
        float continueChance = max(max(kd.x(), kd.y()), kd.z());
        std::uniform_real_distribution<float> udist(0.0f, 1.0f);
        float randomFloat = udist(rng);
        if (randomFloat > continueChance) // terminate path
            return color + direct_light;
        
        // path not terminated
        // sample a random angle based on cosine distribution
        float sin_theta = sqrtf(udist(rng));
        float cos_theta = sqrtf(1-sin_theta*sin_theta);

        //random in plane angle
        float psi = udist(rng)*2.0f*3.1415926535f;

        // calculate tangents
        vec3f tangent = cross(normal, vec3f(0,1,0));
        if (tangent.length2() == 0.0f)
            tangent = cross(normal, vec3f(0,0,1));
        vec3f bitangent = cross(normal, tangent);

        // calculate vector components
        float a = sin_theta*cosf(psi);
        float b = sin_theta*sinf(psi);
        float c = cos_theta;

        // calculate reflected ray direction
        vec3f new_dir = a * tangent + b * bitangent + c * normal;

        color += direct_light + kd * trace(m, rng, hitPos + normal*bias, new_dir, depth+1)/continueChance;
    } else {
        for (Light const& lght: rtc.lights) {
            float tnear2 = F_INFINITY;
            MeshIntersection inter2 = m.intersect(hitPos+(bias*snormal), lght.position-hitPos, tnear2);
            if (!inter2.intersected())
                color += inter.material().color(dir, normal, hitPos, lght, uv);
        }
        // TODO: Tail recursion?
        if (depth < this->max_depth) {
            // TODO: recognize the edge case where normal is in the opposite direction from surfaceNormal
            color += inter.material().reflectivity(uv) * trace(m, rng, hitPos + normal*bias, reflect(dir, normal), depth+1);
        }
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

void RayTracer::savePNG(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image, bool normalize)
{
    struct RGB {
        unsigned char r,g,b,a;
    };
    unsigned height = rtc.yres;
    unsigned width = rtc.xres;
    Array2D<RGB> exporter(height, width);

    vec3f maxi(1.0f);
    if (normalize) {
        maxi = vec3f(0.0f);
        for (unsigned h = 0; h < height; h++)
            for (unsigned w = 0; w < width; w++)
                maxi = max(maxi, image[h][w]);
    }
    float inv_scale = 1.0f/max(maxi.x(), max(maxi.y(), maxi.z()));

    for (unsigned h = 0; h < height; h++) {
        for (unsigned w = 0; w < width; w++) {
            exporter[h][w].r = min(1.0f, image[h][w].x()*inv_scale)*255;
            exporter[h][w].g = min(1.0f, image[h][w].y()*inv_scale)*255;
            exporter[h][w].b = min(1.0f, image[h][w].z()*inv_scale)*255;
            exporter[h][w].a = 255;
        }
    }

    int ret = stbi_write_png(path_to_image.c_str(), width, height, 4, exporter.begin(), width*sizeof(RGB));
    if (ret == 0) {
        std::cout << "Error ocurred when saving file to " << path_to_image << std::endl;
    }
}

void RayTracer::saveHDR(const std::string& path_to_image, const std::vector<std::vector<vec3f> > &image)
{
    unsigned height = rtc.yres;
    unsigned width = rtc.xres;
    Array2D<vec3f_compact> exporter(height, width);

    for (unsigned h = 0; h < height; h++) {
        for (unsigned w = 0; w < width; w++) {
            exporter[h][w] = vec3f_compact(image[h][w]);
        }
    }
    int ret = stbi_write_hdr(path_to_image.c_str(), width, height, 3, (float*)exporter.begin());
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
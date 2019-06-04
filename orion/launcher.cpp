/**
 * This file file serves as an entry point in a standalone raytracer
 * Compile with -DRAYTRACER_STANDALONE to use this
 **/

#ifdef RAYTRACER_STANDALONE

#include <cstdio>
#include <string>

#include <CLI11.hpp>

#include <orion/raytracer.hpp>

int main(int argc, char** argv)
{
    CLI::App raytracer{"Simple AVX raytracer which uses rtc files and magic to create renders."};

    std::string rtc_file;
    raytracer.add_option("rtc_file", rtc_file, "Path to an .rtc file")->required();

    std::string output;
    raytracer.add_option("--output,-o", output, "Output file where rendered image will be saved")->default_str("raytracer.png");

    unsigned supersamples;
    raytracer.add_option("-p", supersamples, "Pixel samples")->default_val("1");

    unsigned lightsamples;
    raytracer.add_option("-l", lightsamples, "Shadow Ray samples")->default_val("1");

    unsigned threads;
    raytracer.add_option("--threads,-t", threads, "How many threads should raytracer use [0 => as many as CPU cores]")->default_val("0");

    CLI11_PARSE(raytracer, argc, argv);

    // if (argc != 2) {
    //     printf("Usage:\nraytracer path/to/rtc/file.rtc\n");
    //     return 1;
    // }

    orion::RayTracer RT;
    RT.traceRTC(rtc_file, output, supersamples, lightsamples, threads);

    return 0;
}

#endif
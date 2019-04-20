#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <stb_image.h>

#include <orion/math.hpp>

namespace orion {

class Texture {
public:
    // Create black 1x1 texture
    // @param color: color of the single-pixel texture
    Texture(vec3f color = vec3f(0.0f)) :
        data(new vec3f[1]{color}),
        width(1), height(1), components(0)
    {}

    // Initialize texture with image
    explicit Texture(const std::string& filename) {
        loadFromFile(filename);
    }

    ~Texture() = default;

    /** @brief Load texture from file specified by filename
     *  @param filename: path to file from which texture should be loaded
     *  @returns 0 on success, -1 otherwise
     **/
    int loadFromFile(const std::string& filename) {
        unsigned char* raw_data = stbi_load(filename.c_str(), &(this->width), &(this->height), &(this->components), 0);
        if (!raw_data) {
            std::cerr << "stbi_load returned a NULL pointer! Path: " << filename << std::endl;
            return -1;
        }
        if (components == 2) {
            std::cerr << "Unknown texture format (2 color channels) in path: " << filename << std::endl;
            return -1;
        }
        m_path = filename;
        auto vec3f_data = new vec3f[width*height];
        data = std::shared_ptr<vec3f[]>(vec3f_data); // let smart pointer manage the data
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int index;
                switch (components) {
                    case 1:
                        vec3f_data[i*width + j] = vec3f(raw_data[i*width + j]/255.0f);
                        break;
                    case 3:
                        index = 3*(i*width + j);    
                        vec3f_data[i*width + j] = vec3f(raw_data[index]/255.0f,
                                                  raw_data[index+1]/255.0f,
                                                  raw_data[index+2]/255.0f);
                        break;
                    case 4:
                        index = 4*(i*width + j);    
                        vec3f_data[i*width + j] = vec3f(raw_data[index]/255.0f,
                                                  raw_data[index+1]/255.0f,
                                                  raw_data[index+2]/255.0f); // skipping the alpha channel
                }
            }
        }

        stbi_image_free(raw_data);
        return 0;
    }

    vec3f color(float x, float y) const {
        int ui = x * width;
        int vi = y * height;

        ui %= width;
        vi %= height;

        if (ui < 0)
            ui = width+ui;

        if (vi < 0)
            vi = height+ui;

        return data[width*vi + ui];
    }

    vec3f color(vec2f uv) const {
        return color(uv.x(), uv.y());
    }

    // isSolidColor can be used to check whether texture is a 1x1 solid color
    bool isSolidColor() const { return components == 0; }

    const std::string& filepath() const { return m_path; }

private:
    std::string m_path;
    std::shared_ptr<vec3f[]> data;
    int width, height, components;
};

}; // namespace orion
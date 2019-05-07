#ifndef ORION_RTC_PARSER_HPP
#define ORION_RTC_PARSER_HPP

#include <vector>
#include <string>

#include <orion/math.hpp>
#include <orion/light.hpp>

namespace orion {
    struct rtc_data {
        int recursion_level;
        int xres;
        int yres;
        float aspect_ratio;

        vec3f view_point;
        vec3f look_at;
        vec3f vector_up;  // default: [0,1,0]
        /*
        vertical view angle defined by proportion of image height to focal distance.
        Default value yview=1.0 gives an angle like in classic camera with image sensor of height y=24mm and focal length f=24mm.
        For yview=0.5 we have normal lens (not wide angle not tele) with f=48 y=24.
        Angles in x direction are defined by yangle and image aspect ratio (inferred from image dimensions). 
        */
        float y_view;
        std::vector<Light> lights;

        std::string obj_file;
        std::string texture_file;
    };

    rtc_data parse_rtc(const std::string& path);
    
    void write_rtc(const std::string& path, const rtc_data& rtc);
};

#endif // ORION_RTC_PARSER_HPP
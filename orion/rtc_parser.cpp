#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>

#include <orion/rtc_parser.hpp>

using namespace std;

namespace orion {

// isspace returns true if character is a whitespace character (this includes control characters)
static bool isspace(char c) {
    return c <= 32;
}

template <typename Iterator, typename BooleanFunction>
static inline Iterator find_if(Iterator begin, Iterator end, BooleanFunction f) {
    Iterator it = begin;
    while (it != end && !f(*it)) {
        ++it;
    }
    return it;
}

// trim from start
static inline string ltrim(string s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) {
        return !isspace(ch);
    }));
    return s;
}

// trim from end
static inline string rtrim(string s) {
    s.erase(find_if(s.rbegin(), s.rend(), [](int ch) {
        return !isspace(ch);
    }).base(), s.end());
    return s;
}

// trim from both ends
static inline string trim(std::string s) {
    s = ltrim(s);
    s = rtrim(s);
    return s;
}

static vec3f load_vec3(istringstream& iss) {
    float x, y, z;
    iss >> x >> y >> z;
    return vec3f(x, y, z);
}

rtc_data parse_rtc(const char* path) 
{
    rtc_data rtc;

    ifstream infile(path);
    if (!infile) {
        printf("Error opening file %s\n", path);
        return rtc;
    }

    string line;
    int line_count = 0;
    int empty_lines = 0;
    while(getline(infile, line))
    {
        line = trim(line);
        if (line[0] != '#' and !line.empty()) {
            istringstream iss(line);
            line_count++;
            if (line_count == 1) {
                rtc.obj_file = line;
            } else if (line_count == 2) {
                rtc.texture_file = line;
            } else if (line_count == 3) { // recursion level
                iss >> rtc.recursion_level;
            } else if (line_count == 4) { // resolution
                iss >> rtc.xres >> rtc.yres;
                rtc.aspect_ratio = float(rtc.xres) / float(rtc.yres);
            } else if (line_count == 5) { // view point
                vec3f vp = load_vec3(iss);
                rtc.view_point = vp;
            } else if (line_count == 6) { // look at
                vec3f la = load_vec3(iss);
                rtc.look_at = la;
            } else if (line_count == 7) { // vector up
                vec3f up = load_vec3(iss);
                rtc.vector_up = up;
            } else if (line_count == 8) { // y fov
                iss >> rtc.y_view;
            } else {
                char c;
                iss >> c;
                if (c != 'L' and c != 'l') {
                    printf("Invalid character '%c' in line %d `%s`\n", c, line_count + empty_lines, line.c_str());
                } else {
                    vec3f pos = load_vec3(iss);
                    vec3f col = load_vec3(iss);
                    col /= 255; // Entry is in 0-255 rgb
                    float intensity;
                    iss >> intensity;

                    Light l;
                    l.position = pos;
                    l.color= col;
                    l.intensity = intensity;

                    rtc.lights.push_back(l);
                }
            }
        } else { // Skip line if empty or comment
            empty_lines++;
        } 
    }

    return rtc;
}

};
# Orion
Simple raytracer.

# Requirements
## general
- OpenGL 3.3 compatible hardware*
- g++ (with c++17 support)
- make
## libs
- GLEW*
- glfw3*
- GL*
- GLU*
- assimp
## other
- GLM*
- GLAD* (Included in this repository for GL 3.3 Core)
- tqdm.cpp (See Setup below)

`*` - denotes libraries required by viewer and are not necessary for the raytracer.

`NOTE:` If you are compiling libraries from sources remember to modify `LDPATHS` in Makefile to contain paths with compiled libs.

# Building and running
## Setup
1. Download tqdm.cpp submodule
    
    `@> git submodule update --init --recursive`
## Raytracer
1. `@> make raytracer`
2. `@> ./build/raytracer assets/nanosuit.rtc`
## Viewer
1. `@> make viewer`
2. `@> ./build/rviewer`

# To do
shallow BVH
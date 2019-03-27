# Orion
As the rays are cast by Orion, the surfaces are lit by the power of raytracing

# Requirements
## general
- OpenGL 3.3 compatible hardware
- g++8
- make
## libs
- GLEW 
- glfw3
- GL
- GLU
- assimp
## other
- GLM
- GLAD (Included in this repository for GL 3.3 Core)

`NOTE:` If you are compiling libraries from sources remember to modify `LDPATHS` in Makefile to contain paths with compiled libs

# Building and running
1. `@> make`
2. `@> ./build/app`

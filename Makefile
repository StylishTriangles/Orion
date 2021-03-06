BUILD_DIR := build

ORION_DIR := orion
ORION_OBJ_DIR := $(BUILD_DIR)/$(ORION_DIR)
ORION_SRC_FILES := $(shell find $(ORION_DIR) -name "*.cpp")
ORION_OBJ_FILES := $(patsubst $(ORION_DIR)/%,$(ORION_OBJ_DIR)/%.o,$(ORION_SRC_FILES))

VIEWER_DIR := viewer
VIEWER_OBJ_DIR := $(BUILD_DIR)/$(VIEWER_DIR)
VIEWER_SRC_FILES := $(shell find $(VIEWER_DIR) -name "*.cpp" -or -name "*.c")
VIEWER_OBJ_FILES := $(patsubst $(VIEWER_DIR)/%,$(VIEWER_OBJ_DIR)/%.o,$(VIEWER_SRC_FILES))

ORION_DEPS_SRC := vendor/stb_image.cpp vendor/stb_image_write.cpp
ORION_DEPS_OBJ := $(patsubst %,$(BUILD_DIR)/%.o,$(ORION_DEPS_SRC))

VIEWER_DEPS_SRC := vendor/glad/glad.c $(ORION_DEPS_SRC)
VIEWER_DEPS_OBJ := $(patsubst %,$(BUILD_DIR)/%.o,$(VIEWER_DEPS_SRC))

ARCH := native
CC := g++
LDFLAGS := -lGLEW -lglfw3 -lGL -lGLU -lassimp -ldl -lstdc++fs
LDPATHS := -L/usr/lib64
CPPFLAGS := -O3 -MMD -g -mavx -mfma -mbmi -fopenmp -march=$(ARCH)
CXXFLAGS := -Wall -Wextra -std=c++17
CFLAGS := 
INCLUDE_PATHS := -I. -Ivendor

LDFLAGS_RAYTRACER := -lassimp -ldl -lstdc++fs
LDFLAGS_VIEWER := -lGLEW -lglfw3 -lGL -lGLU -lassimp -ldl -lstdc++fs

# debug settings
BUILD_DIR_DEBUG := build_debug
CPPFLAGS_DEBUG := -g -Og -MMD -msse4.1
CXXFLAGS_DEBUG := -Wall -Wextra -std=c++17
CFLAGS_DEBUG := 

all: raytracer viewer
	@true

orion: $(ORION_OBJ_FILES)
	@true

viewer: $(ORION_OBJ_FILES) $(VIEWER_OBJ_FILES) $(VIEWER_DEPS_OBJ)
	$(CC) $(CPPFLAGS) -o $(BUILD_DIR)/rviewer $^ $(LDPATHS) $(LDFLAGS_VIEWER)

raytracer: $(ORION_OBJ_DIR)/raytracer_launcher.o $(ORION_OBJ_FILES) $(ORION_DEPS_OBJ)
	$(CC) $(CPPFLAGS) -o $(BUILD_DIR)/raytracer $^ $(LDPATHS) $(LDFLAGS_RAYTRACER)

$(ORION_OBJ_DIR)/raytracer_launcher.o:
	@mkdir $(@D) -p
	$(CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE_PATHS) -c -o $@ -DRAYTRACER_STANDALONE $(ORION_DIR)/launcher.cpp

# general rule for building .o files
$(BUILD_DIR)/%.o: %
	@mkdir $(@D) -p
	$(CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE_PATHS) -c -o $@ $<

debug:
	@echo "# This file was automatically generated with 'make debug'" | cat - Makefile > Makefile_debug
	@sed -i -r 's/^(BUILD_DIR :=).*/\1 $(BUILD_DIR_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CPPFLAGS :=).*/\1 $(CPPFLAGS_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CXXFLAGS :=).*/\1 $(CXXFLAGS_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CFLAGS :=).*/\1 $(CFLAGS_DEBUG)/' Makefile_debug
	@echo "Successfully created Makefile_debug, run with 'make -f Makefile_debug'"

clean:
	rm -rf $(BUILD_DIR)

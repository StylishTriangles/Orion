BUILD_DIR := build
SRC_DIR := src
OBJ_DIR := $(BUILD_DIR)/obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))
CMN_DIR := common
CMN_OBJ_DIR := $(BUILD_DIR)/common
CMN_FILES := $(wildcard $(CMN_DIR)/*.cpp)
CMN_OBJ_FILES := $(patsubst $(CMN_DIR)/%.cpp,$(CMN_OBJ_DIR)/%.o,$(CMN_FILES))

CC := g++
LDFLAGS := -lGLEW -lglfw3 -lGL -lGLU -lassimp
LDPATHS := -L/usr/lib64
CPPFLAGS := -O3 -flto -fuse-linker-plugin -MMD
CXXFLAGS := -Wall -std=c++17
INCLUDE_PATHS := -I.

# debug settings
BUILD_DIR_DEBUG := build_debug
CPPFLAGS_DEBUG := -g -Og
CXXFLAGS_DEBUG := -Wall -Wextra -std=c++17

all: create_build_dirs $(BUILD_DIR)/app
	@true

debug:
	@cp Makefile Makefile_debug
	@sed -i -r 's/^(BUILD_DIR :=).*/\1 $(BUILD_DIR_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CPPFLAGS :=).*/\1 $(CPPFLAGS_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CXXFLAGS :=).*/\1 $(CXXFLAGS_DEBUG)/' Makefile_debug
	@sed -i "1s/^/# This file was automatically generated with 'make debug'\n/" Makefile_debug
	@echo "Created Makefile_debug, run with 'make -f Makefile_debug'"

create_build_dirs:
	mkdir $(BUILD_DIR) -p
	mkdir $(BUILD_DIR)/obj -p
	mkdir $(BUILD_DIR)/common -p

$(BUILD_DIR)/app: $(OBJ_FILES) $(CMN_OBJ_FILES)
	$(CC) $(CPPFLAGS) -o $@ $^ $(LDPATHS) $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE_PATHS) -c -o $@ $<

$(CMN_OBJ_DIR)/%.o: $(CMN_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE_PATHS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
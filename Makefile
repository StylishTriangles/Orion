BUILD_DIR := build
SRC_DIR := src
OBJ_DIR := $(BUILD_DIR)/obj
SRC_FILES := $(shell find src -name *.cpp -or -name *.c)
OBJ_FILES := $(shell echo "$(SRC_FILES)" | sed -E 's;$(SRC_DIR)/(\S*)(\.cpp|\.c);$(OBJ_DIR)/\1.o;g')
CMN_DIR := common
CMN_OBJ_DIR := $(BUILD_DIR)/common
CMN_FILES := $(wildcard $(CMN_DIR)/*.cpp)
CMN_OBJ_FILES := $(patsubst $(CMN_DIR)/%.cpp,$(CMN_OBJ_DIR)/%.o,$(CMN_FILES))

CC := g++
LDFLAGS := -lGLEW -lglfw3 -lGL -lGLU -lassimp -ldl
LDPATHS := -L/usr/lib64
CPPFLAGS := -O3 -flto -fuse-linker-plugin -MMD
CXXFLAGS := -Wall -std=c++17
CFLAGS := 
INCLUDE_PATHS := -I. -I./src

# debug settings
BUILD_DIR_DEBUG := build_debug
CPPFLAGS_DEBUG := -g -Og -MMD
CXXFLAGS_DEBUG := -Wall -Wextra -std=c++17
CFLAGS_DEBUG := 

all: $(BUILD_DIR)/app
	@true

debug:
	@echo "# This file was automatically generated with 'make debug'" | cat - Makefile > Makefile_debug
	@sed -i -r 's/^(BUILD_DIR :=).*/\1 $(BUILD_DIR_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CPPFLAGS :=).*/\1 $(CPPFLAGS_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CXXFLAGS :=).*/\1 $(CXXFLAGS_DEBUG)/' Makefile_debug
	@sed -i -r 's/^(CFLAGS :=).*/\1 $(CFLAGS_DEBUG)/' Makefile_debug
	@echo "Successfully created Makefile_debug, run with 'make -f Makefile_debug'"

$(BUILD_DIR)/app: $(OBJ_FILES) $(CMN_OBJ_FILES)
	$(CC) $(CPPFLAGS) -o $@ $^ $(LDPATHS) $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir $(@D) -p
	$(CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE_PATHS) -c -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir $(@D) -p
	$(CC) $(CPPFLAGS) $(CFLAGS) $(INCLUDE_PATHS) -c -o $@ $<

$(CMN_OBJ_DIR)/%.o: $(CMN_DIR)/%.cpp
	@mkdir $(@D) -p
	@# Disable warnings during compilation of "common" files
	$(CC) $(CPPFLAGS) $(CXXFLAGS) -w $(INCLUDE_PATHS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
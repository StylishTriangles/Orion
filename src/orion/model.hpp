#ifndef ORION_MODEL_HPP
#define ORION_MODEL_HPP

#include <string>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <orion/mesh.hpp>

namespace orion {

// TracedModel is a class representing a 3D model imported with the Assimp library.
// source code is based on the Model class with some adaptations to make it easier to use in raytracing.
// This class uses name TracedModel to distinguish it from other Model type.
class TracedModel
{
public:
    /*  Model Data */

    // Temporarily removed as we are not utilizing textures
    // std::vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<TracedMesh> meshes;
    std::string directory;
    bool gammaCorrection;

    /*  Functions   */
    // constructor, default
    TracedModel() = default;

    // constructor, expects a filepath to a 3D model.
    TracedModel(std::string const &path, bool gamma = false) : gammaCorrection(gamma)
    {
        loadModel(path);
    }

    bool intersect(const vec3f &orig, 
                   const vec3f &dir,
                   float &t,
                   vec3f& color) const
    {
        for (TracedMesh const& tm: meshes) {
            // float u,v; // we will not use these atm
            const Triangle* tri = tm.intersect(orig, dir, t);
            if (tri) {
                color = tm.baseColor.color_ambient;
                return true;
            }
        }
        return false;
    }
    
private:
    /*  Functions   */
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    TracedMesh processMesh(aiMesh *mesh, const aiScene *scene);
};

};

#endif // ORION_MODEL_HPP
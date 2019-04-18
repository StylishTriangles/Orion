#ifndef ORION_MODEL_HPP
#define ORION_MODEL_HPP

#include <string>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <orion/material.hpp>
#include <orion/mesh.hpp>

namespace orion {

class MeshIntersection {
public:
    MeshIntersection() : pMesh(nullptr) {}

    vec3f normal() const { return pMesh->normal(triangleID, u, v); }
    const Material& material() const { return pMesh->material(); }
    bool intersected() const { return pMesh != nullptr; }
    
    /** Members **/
    const TracedMesh* pMesh;
    unsigned int triangleID;
    float u, v;
}; // class MeshIntersection

// TracedModel is a class representing a 3D model imported with the Assimp library.
// source code is based on the Model class with some adaptations to make it easier to use in raytracing.
// This class uses name TracedModel to distinguish it from other Model type.
class TracedModel
{
public:
    /*  Functions   */
    // constructor, default
    TracedModel() = default;

    // constructor, expects a filepath to a 3D model.
    TracedModel(std::string const &path, bool gamma = false) : gammaCorrection(gamma)
    {
        loadModel(path);
    }

    MeshIntersection intersect(const vec3f& origin, const vec3f &dir, float &t) {
        MeshIntersection ret;
        for (TracedMesh const& tm: meshes) {
            unsigned int triangleID = tm.intersect(origin, dir, t, ret.u, ret.v);
            if (triangleID != INVALID_INTERSECT_ID) {
                ret.pMesh = &tm;
                ret.triangleID = triangleID;
            }
        }
        return ret;
    }

private:
    /*  Model Data */

    // Temporarily removed as we are not utilizing textures
    // std::vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<TracedMesh> meshes;
    std::string directory;
    bool gammaCorrection;

    /*  Functions   */
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    TracedMesh processMesh(aiMesh *mesh, const aiScene *scene);
};

};

#endif // ORION_MODEL_HPP
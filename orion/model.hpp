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

// class MeshIntersection {
// public:
//     MeshIntersection() : pMesh(nullptr) {}

//     vec3f normal() const { 
//         // if (material().hasBumpMap())
//             // return material().normalBumpMap(surfaceNormal(), pMesh->tangent(triangleID, uv[0], uv[1]), pMesh->bitangent(triangleID, uv[0], uv[1]), texture_uv());
//         return surfaceNormal();
//     }
//     vec3f surfaceNormal() const { return pMesh->normal(triangleID, uv[0], uv[1]); }
//     vec2f texture_uv() const { return pMesh->texture_uv(triangleID, uv[0], uv[1]); }
//     const Material& material() const { return pMesh->material(); }
//     bool intersected() const { return pMesh != nullptr; }
    
//     /** Members **/
//     const TracedMesh* pMesh;
//     unsigned int triangleID;
//     vec2f uv;
// }; // class MeshIntersection

// TracedModel is a class representing a 3D model imported with the Assimp library.
// source code is based on the Model class with some adaptations to make it easier to use in raytracing.
// This class uses name TracedModel to distinguish it from other Model type.
class TracedModel : public Primitive
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

    // MeshIntersection intersect(const vec3f& origin, const vec3f &dir, float &t) {
    //     MeshIntersection ret;
    //     for (TracedMesh const& tm: meshes) {
    //         unsigned int triangleID = tm.intersect(origin, dir, t, ret.uv[0], ret.uv[1]);
    //         if (triangleID != INVALID_INTERSECT_ID) {
    //             ret.pMesh = &tm;
    //             ret.triangleID = triangleID;
    //         }
    //     }
    //     return ret;
    // }

    int triangleCount() {
        int sum = 0;
        for (auto const& mesh: meshes) {
            sum += mesh.triangleCount();
        }
        return sum;
    }

    vec3f lowerBound() const {
        vec3f mini(F_INFINITY);
        for (auto const& mesh: meshes) {
            mini = min(mini, mesh.lowerBound());
        }
        return mini;
    }

    vec3f upperBound() const {
        vec3f maxi(-F_INFINITY);
        for (auto const& mesh: meshes) {
            maxi = max(maxi, mesh.upperBound());
        }
        return maxi;
    }

private:
    /*  Model Data */

    // Temporarily removed as we are not utilizing textures
    std::vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<TracedMesh> meshes;
    std::string directory;
    bool gammaCorrection;

    /*  Functions   */
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    TracedMesh processMesh(aiMesh *mesh, const aiScene *scene);

    std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type);
};

template<>
class Intersector<TracedModel> {
public:
    template<unsigned size>
    using Intersection = Intersector<TracedMesh>::Intersection;

    template<unsigned size>
    void intersect (
        const PackedRay<size>& r,
        const TracedModel& model,
        Intersection<size>& res
    ) {
        Intersector<TracedMesh> inter;
        for (unsigned i = 0; i < model.meshes.size(); i++) {
            inter.intersect(r, model.meshes[i], res);
        }
    }
};

}; // namespace orion

#endif // ORION_MODEL_HPP
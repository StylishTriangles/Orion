#include <iostream>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <orion/geometry.hpp>
#include <orion/material.hpp>
#include <orion/model.hpp>

#include <cassert>

using namespace std;

namespace orion {

// loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
void TracedModel::loadModel(string const &path)
{
    // read file via ASSIMP
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path, 
        aiProcess_JoinIdenticalVertices |
        aiProcess_CalcTangentSpace |
        aiProcess_Triangulate |
        aiProcess_FlipUVs | 
        aiProcess_GenNormals);
    // check for errors
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
        cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
        return;
    }
    // reduce copy count optimizations
    textures_loaded.reserve(scene->mNumTextures); 
    meshes.reserve(scene->mNumMeshes);
    // retrieve the directory path of the filepath
    directory = path.substr(0, path.find_last_of('/'));
    // process ASSIMP's root node recursively
    processNode(scene->mRootNode, scene);
}

// processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
void TracedModel::processNode(aiNode *node, const aiScene *scene)
{
    // process each mesh located at the current node
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        // the node object only contains indices to index the actual objects in the scene. 
        // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes.push_back(processMesh(mesh, scene));
    }
    // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++)
    {
        processNode(node->mChildren[i], scene);
    }

}

TracedMesh TracedModel::processMesh(aiMesh *mesh, const aiScene *scene)
{
    const int INDICES_PER_FACE = 3;
    // data to fill
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    // reserve space for data
    vertices.reserve(mesh->mNumVertices);
    indices.reserve(mesh->mNumFaces*INDICES_PER_FACE);

    // Walk through each of the mesh's vertices
    for(unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        Vertex vertex;
        vec3f vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
        // positions
        vector = vec3f(mesh->mVertices[i].x,
                       mesh->mVertices[i].y,
                       mesh->mVertices[i].z);
        vertex.position = vector;
        // normals
        vector = vec3f(mesh->mNormals[i].x,
                       mesh->mNormals[i].y,
                       mesh->mNormals[i].z);
        vertex.normal = vector;
        // texture coordinates
        if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
        {
            vec2f vec;
            // a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't 
            // use models where a vertex can have multiple texture coordinates so we always take the first set (0).
            vec.x() = mesh->mTextureCoords[0][i].x; 
            vec.y() = mesh->mTextureCoords[0][i].y;
            vertex.texCoords = vec;
        }
        else
            vertex.texCoords = vec2f(0.0f, 0.0f);
        
        if (mesh->HasTangentsAndBitangents()) {
            vector = vec3f(mesh->mTangents[i].x,
                           mesh->mTangents[i].y,
                           mesh->mTangents[i].z);
            // vertex.tangent = orthogonalize(vertex.normal, vector).normalized();
            vertex.tangent = vector;
            // vertex.tangent = orthogonalize(vertex.normal, vertex.tangent).normalized();

            vector = vec3f(mesh->mBitangents[i].x,
                           mesh->mBitangents[i].y,
                           mesh->mBitangents[i].z);
            vertex.bitangent = vector;
            // vertex.bitangent = cross(vertex.normal, vertex.tangent);
        } else {
            vertex.tangent = vec3f(10.0f);
            vertex.bitangent = vec3f(10.0f);
        }
        
        vertices.push_back(vertex);
    }
    // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        assert(face.mNumIndices == 3);
        // retrieve all indices of the face and store them in the indices vector
        for(unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }
    // process materials
    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    
    // we assume a convention for sampler names in the shaders. Each diffuse texture should be named
    // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER. 
    // Same applies to other texture as the following list summarizes:
    // diffuse: texture_diffuseN
    // specular: texture_specularN
    // normal: texture_normalN


    // 1. diffuse maps
    vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE);
    // 2. specular maps
    vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR);
    // 3. normal maps
    std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT);
    // 4. height maps
    std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT);


    aiColor3D Ka, Kd, Ks, Ke;
    float shininess, opacity;
    material->Get(AI_MATKEY_COLOR_AMBIENT, Ka);
    material->Get(AI_MATKEY_COLOR_DIFFUSE, Kd);
    material->Get(AI_MATKEY_COLOR_SPECULAR, Ks);
    material->Get(AI_MATKEY_COLOR_EMISSIVE, Ke);
    material->Get(AI_MATKEY_SHININESS, shininess);
    material->Get(AI_MATKEY_OPACITY, opacity);

    auto aiToV3F = [](aiColor3D& col) -> vec3f {
        return vec3f(col.r, col.g, col.b);
    };
    SolidSurface base;
    // Apply base surface colors
    base.color_ambient = aiToV3F(Ka);
    base.color_diffuse = aiToV3F(Kd);
    base.color_specular = aiToV3F(Ks);
    base.color_emissive = aiToV3F(Ke);
    base.shininess = shininess;
    base.opacity = opacity;

    Material m(base);
    // Save textures in the material if
    if (!diffuseMaps.empty())
        m.setTexture(Material::TextureType::DIFFUSE, diffuseMaps[0]);
    if (!specularMaps.empty())
        m.setTexture(Material::TextureType::SPECULAR, diffuseMaps[0]);
    if (!normalMaps.empty())
        m.setTexture(Material::TextureType::NORMAL, normalMaps[0]);
    
    // return a mesh object created from the extracted mesh data
    return TracedMesh(vertices, indices, m);
}

vector<Texture> TracedModel::loadMaterialTextures(aiMaterial *mat, aiTextureType type)
{
    vector<Texture> textures;
    if (mat->GetTextureCount(type) > 1) {
        std::cout << "Warning! Multiple textures of same type present, the first one fetched will be used.\n";
    }
    for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);

        std::string texturePath = this->directory + "/" +  std::string(str.C_Str());

        // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
        bool skip = false;
        for(unsigned int j = 0; j < textures_loaded.size(); j++)
        {
            if(textures_loaded[j].filepath() == texturePath)
            {
                textures.push_back(textures_loaded[j]);
                skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                break;
            }
        }
        if(!skip)
        {   // if texture hasn't been loaded already, load it
            Texture texture(texturePath);
            textures.push_back(texture);
            textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
        }
    }
    return textures;
}

}; // namespace orion

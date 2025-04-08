#ifndef MODEL_H
#define MODEL_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <limits>

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
};

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices)
    {
        this->vertices = vertices;
        this->indices = indices;
    }

    void Draw() const
    {
        if (vertices.empty() || indices.empty()) {
            std::cout << "Warning: Trying to draw empty mesh" << std::endl;
            return;
        }
        
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        
        glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vertices[0].Position);
        glNormalPointer(GL_FLOAT, sizeof(Vertex), &vertices[0].Normal);
        
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, &indices[0]);
        
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
    }
};

class Model 
{
public:
    std::vector<Mesh> meshes;
    std::string directory;
    bool gammaCorrection;

    Model() : gammaCorrection(false) {}

    Model(std::string const &path, bool gamma = false) : gammaCorrection(gamma)
    {
        loadModel(path);
        normalizeModelScale();
        std::cout << "Model loaded and normalized" << std::endl;
    }

    void Draw() const
    {
        if (meshes.empty()) {
            std::cout << "Warning: No meshes to draw" << std::endl;
            return;
        }
        
        for (const auto& mesh : meshes) {
            mesh.Draw();
        }
    }
    
private:
    glm::vec3 minBounds;
    glm::vec3 maxBounds;
    
    void loadModel(std::string const &path)
    {
        minBounds = glm::vec3(std::numeric_limits<float>::max());
        maxBounds = glm::vec3(std::numeric_limits<float>::lowest());
        
        Assimp::Importer importer;
        unsigned int importFlags = aiProcess_Triangulate | 
                                 aiProcess_GenSmoothNormals | 
                                 aiProcess_FlipUVs | 
                                 aiProcess_JoinIdenticalVertices |
                                 aiProcess_PreTransformVertices;
                                 
        const aiScene* scene = importer.ReadFile(path, importFlags);
        
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cout << "ASSIMP Error: " << importer.GetErrorString() << std::endl;
            return;
        }
        
        std::cout << "ASSIMP: Successfully loaded scene with " << scene->mNumMeshes << " meshes" << std::endl;
        
        directory = path.substr(0, path.find_last_of("/\\"));
        processNode(scene->mRootNode, scene);
        
        size_t totalVertices = 0, totalIndices = 0;
        for (const auto& mesh : meshes) {
            totalVertices += mesh.vertices.size();
            totalIndices += mesh.indices.size();
        }
        std::cout << "Model loaded: " << meshes.size() << " meshes, " 
                  << totalVertices << " vertices, " 
                  << totalIndices << " indices" << std::endl;
    }

    void processNode(aiNode *node, const aiScene *scene)
    {
        for(unsigned int i = 0; i < node->mNumMeshes; i++)
        {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(mesh, scene));
        }
        
        for(unsigned int i = 0; i < node->mNumChildren; i++)
        {
            processNode(node->mChildren[i], scene);
        }
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene)
    {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;

        for(unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            glm::vec3 vector;
            
            vector.x = mesh->mVertices[i].x;
            vector.y = mesh->mVertices[i].y;
            vector.z = mesh->mVertices[i].z;
            vertex.Position = vector;
            
            minBounds.x = std::min(minBounds.x, vector.x);
            minBounds.y = std::min(minBounds.y, vector.y);
            minBounds.z = std::min(minBounds.z, vector.z);
            maxBounds.x = std::max(maxBounds.x, vector.x);
            maxBounds.y = std::max(maxBounds.y, vector.y);
            maxBounds.z = std::max(maxBounds.z, vector.z);
            
            if (mesh->HasNormals())
            {
                vector.x = mesh->mNormals[i].x;
                vector.y = mesh->mNormals[i].y;
                vector.z = mesh->mNormals[i].z;
                vertex.Normal = vector;
            }
            else
            {
                vertex.Normal = glm::vec3(0.0f, 0.0f, 1.0f);
            }
            
            vertices.push_back(vertex);
        }
        
        for(unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            for(unsigned int j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }
        
        return Mesh(vertices, indices);
    }
    
    void normalizeModelScale()
    {
        if (meshes.empty()) {
            std::cout << "Warning: No meshes loaded to normalize" << std::endl;
            return;
        }
        
        glm::vec3 size = maxBounds - minBounds;
        float maxDim = std::max(std::max(size.x, size.y), size.z);
        
        if (maxDim <= 0.0f) {
            std::cout << "Warning: Invalid model bounds for normalization" << std::endl;
            return;
        }
        
        float scale = 2.0f / maxDim;
        
        glm::vec3 center = (minBounds + maxBounds) * 0.5f;
        
        std::cout << "Model bounds: Min(" 
                  << minBounds.x << ", " << minBounds.y << ", " << minBounds.z 
                  << "), Max(" 
                  << maxBounds.x << ", " << maxBounds.y << ", " << maxBounds.z 
                  << ")" << std::endl;
        
        for (auto& mesh : meshes) {
            for (auto& vertex : mesh.vertices) {
                vertex.Position = (vertex.Position - center) * scale;
            }
        }
        
        std::cout << "Model normalized with scale factor: " << scale << std::endl;
    }
};

#endif

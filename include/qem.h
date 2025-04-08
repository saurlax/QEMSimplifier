#ifndef QEM_H
#define QEM_H

#include <glm/glm.hpp>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "model.h"

// Quadric for error calculation
struct Quadric
{
    float a, b, c, d, e, f, g, h, i, j;

    Quadric() : a(0), b(0), c(0), d(0), e(0), f(0), g(0), h(0), i(0), j(0) {}

    Quadric(float a, float b, float c, float d)
    {
        this->a = a * a;
        this->b = a * b;
        this->c = a * c;
        this->d = a * d;
        this->e = b * b;
        this->f = b * c;
        this->g = b * d;
        this->h = c * c;
        this->i = c * d;
        this->j = d * d;
    }

    Quadric operator+(const Quadric &q) const
    {
        Quadric result;
        result.a = a + q.a;
        result.b = b + q.b;
        result.c = c + q.c;
        result.d = d + q.d;
        result.e = e + q.e;
        result.f = f + q.f;
        result.g = g + q.g;
        result.h = h + q.h;
        result.i = i + q.i;
        result.j = j + q.j;
        return result;
    }

    float calculateError(const glm::vec3 &v) const
    {
        return a * v.x * v.x + 2 * b * v.x * v.y + 2 * c * v.x * v.z + 2 * d * v.x +
               e * v.y * v.y + 2 * f * v.y * v.z + 2 * g * v.y +
               h * v.z * v.z + 2 * i * v.z + j;
    }
};

// Edge collapse data structure
struct Edge
{
    int v1, v2;
    float error;
    glm::vec3 target;

    Edge(int v1, int v2, float error, const glm::vec3 &target)
        : v1(v1), v2(v2), error(error), target(target) {}

    bool operator<(const Edge &other) const
    {
        return error > other.error; // Min-heap based on error
    }
};

// QEM Model Simplifier with boundary preservation
class QEM
{
private:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Quadric> quadrics;
    std::vector<bool> vertexRemoved;
    std::vector<std::vector<int>> vertexFaces;
    std::vector<std::vector<int>> vertexConnections;
    std::priority_queue<Edge> edgeHeap;
    std::set<std::pair<int, int>> boundaryEdges;
    float targetRatio;

public:
    QEM(const Model &model) : targetRatio(1.0f)
    {
        if (model.meshes.empty())
            return;

        // Process the first mesh
        const Mesh &mesh = model.meshes[0];
        vertices = mesh.vertices;
        indices = mesh.indices;

        initializeDataStructures();
    }

    Model simplify(float ratio)
    {
        // Ensure we preserve enough faces
        ratio = std::max(0.1f, ratio);
        targetRatio = ratio;

        // Reset data for fresh simplification
        vertexRemoved.assign(vertexRemoved.size(), false);
        edgeHeap = std::priority_queue<Edge>();

        // Identify boundary edges
        boundaryEdges = identifyBoundaryEdges();
        std::cout << "Found " << boundaryEdges.size() << " boundary edges" << std::endl;

        // Weld boundary vertices that are close together
        weldBoundaryVertices(0.001f);

        // Recalculate quadrics and initialize edge heap
        computeQuadrics();
        initializeEdgeHeap();

        int targetFaces = static_cast<int>((indices.size() / 3) * targetRatio);
        int currentFaces = indices.size() / 3;

        std::cout << "Simplifying from " << currentFaces << " to " << targetFaces << " faces" << std::endl;

        // Edge collapse process
        int iterCount = 0;
        int maxIters = currentFaces; // Reasonable iteration limit
        int successfulCollapses = 0;

        while (currentFaces > targetFaces && !edgeHeap.empty() && iterCount < maxIters)
        {
            // Get edge with smallest error
            Edge edge = edgeHeap.top();
            edgeHeap.pop();

            // Skip if vertices already removed
            if (vertexRemoved[edge.v1] || vertexRemoved[edge.v2])
                continue;

            // Skip boundary edges
            if (isBoundaryEdge(edge.v1, edge.v2))
                continue;

            // Skip if collapse would disrupt topology
            if (!isValidCollapse(edge.v1, edge.v2))
                continue;

            // Perform the edge collapse
            collapseEdge(edge.v1, edge.v2, edge.target);
            successfulCollapses++;

            // Enqueue new potential collapses
            for (int v : vertexConnections[edge.v1])
            {
                if (!vertexRemoved[v])
                {
                    glm::vec3 optimalPos;
                    float error = computeEdgeCollapseCost(edge.v1, v, optimalPos);
                    edgeHeap.push(Edge(edge.v1, v, error, optimalPos));
                }
            }

            // Update face count
            currentFaces--;
            iterCount++;
        }

        // Build the simplified model
        Model result;
        result.meshes.push_back(buildSimplifiedMesh());

        std::cout << "Simplification complete. New model has "
                  << result.meshes[0].indices.size() / 3 << " faces" << std::endl;

        return result;
    }

    bool exportToObj(const std::string &filename, float ratio)
    {
        Model simplified = simplify(ratio);
        if (simplified.meshes.empty())
            return false;

        const Mesh &mesh = simplified.meshes[0];

        std::ofstream file(filename);
        if (!file.is_open())
            return false;

        // Write header
        file << "# Simplified mesh\n";
        file << "# Vertices: " << mesh.vertices.size() << "\n";
        file << "# Faces: " << mesh.indices.size() / 3 << "\n\n";

        // Write vertices
        for (const auto &v : mesh.vertices)
        {
            file << "v " << v.Position.x << " " << v.Position.y << " " << v.Position.z << "\n";
        }

        // Write normals
        for (const auto &v : mesh.vertices)
        {
            file << "vn " << v.Normal.x << " " << v.Normal.y << " " << v.Normal.z << "\n";
        }

        // Write faces (OBJ indices are 1-based)
        for (size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            file << "f "
                 << mesh.indices[i] + 1 << "//" << mesh.indices[i] + 1 << " "
                 << mesh.indices[i + 1] + 1 << "//" << mesh.indices[i + 1] + 1 << " "
                 << mesh.indices[i + 2] + 1 << "//" << mesh.indices[i + 2] + 1 << "\n";
        }

        file.close();
        return true;
    }

private:
    void initializeDataStructures()
    {
        int numVertices = vertices.size();
        int numFaces = indices.size() / 3;

        // Initialize vertex-related data
        quadrics.resize(numVertices);
        vertexRemoved.resize(numVertices, false);
        vertexFaces.resize(numVertices);
        vertexConnections.resize(numVertices);

        // Associate vertices with faces
        for (int f = 0; f < numFaces; f++)
        {
            int v1 = indices[f * 3];
            int v2 = indices[f * 3 + 1];
            int v3 = indices[f * 3 + 2];

            vertexFaces[v1].push_back(f);
            vertexFaces[v2].push_back(f);
            vertexFaces[v3].push_back(f);

            // Build vertex connections (edges)
            vertexConnections[v1].push_back(v2);
            vertexConnections[v1].push_back(v3);
            vertexConnections[v2].push_back(v1);
            vertexConnections[v2].push_back(v3);
            vertexConnections[v3].push_back(v1);
            vertexConnections[v3].push_back(v2);
        }

        // Remove duplicate connections
        for (int i = 0; i < numVertices; i++)
        {
            std::sort(vertexConnections[i].begin(), vertexConnections[i].end());
            vertexConnections[i].erase(
                std::unique(vertexConnections[i].begin(), vertexConnections[i].end()),
                vertexConnections[i].end());
        }

        // Compute initial quadrics
        computeQuadrics();
    }

    void computeQuadrics()
    {
        // Reset quadrics
        for (auto &q : quadrics)
            q = Quadric();

        int numFaces = indices.size() / 3;

        for (int f = 0; f < numFaces; f++)
        {
            int v1 = indices[f * 3];
            int v2 = indices[f * 3 + 1];
            int v3 = indices[f * 3 + 2];

            // Skip degenerate triangles
            if (v1 == v2 || v2 == v3 || v3 == v1)
                continue;

            // Calculate plane equation for this face
            glm::vec3 p1 = vertices[v1].Position;
            glm::vec3 p2 = vertices[v2].Position;
            glm::vec3 p3 = vertices[v3].Position;

            glm::vec3 normal = glm::normalize(glm::cross(p2 - p1, p3 - p1));
            float d = -glm::dot(normal, p1);

            // Create quadric from plane
            Quadric q(normal.x, normal.y, normal.z, d);

            // Add to vertex quadrics
            quadrics[v1] = quadrics[v1] + q;
            quadrics[v2] = quadrics[v2] + q;
            quadrics[v3] = quadrics[v3] + q;
        }
    }

    void initializeEdgeHeap()
    {
        for (size_t i = 0; i < vertices.size(); i++)
        {
            if (vertexRemoved[i])
                continue;

            for (int j : vertexConnections[i])
            {
                if (i < j && !vertexRemoved[j]) // Process each edge once
                {
                    // Skip boundary edges
                    if (isBoundaryEdge(i, j))
                        continue;

                    // Compute optimal position and error
                    glm::vec3 optimalPos;
                    float error = computeEdgeCollapseCost(i, j, optimalPos);

                    // Add to edge heap
                    edgeHeap.push(Edge(i, j, error, optimalPos));
                }
            }
        }
    }

    float computeEdgeCollapseCost(int v1, int v2, glm::vec3 &optimalPos)
    {
        // Combine quadrics
        Quadric q = quadrics[v1] + quadrics[v2];

        // For best results, we should solve the linear system:
        // [a b c; b e f; c f h] * [x; y; z] = [-d; -g; -i]
        // For simplicity, we can try midpoint and endpoints
        optimalPos = (vertices[v1].Position + vertices[v2].Position) * 0.5f;

        float midError = q.calculateError(optimalPos);
        float v1Error = q.calculateError(vertices[v1].Position);
        float v2Error = q.calculateError(vertices[v2].Position);

        if (v1Error < midError && v1Error < v2Error)
        {
            optimalPos = vertices[v1].Position;
            return v1Error;
        }
        else if (v2Error < midError)
        {
            optimalPos = vertices[v2].Position;
            return v2Error;
        }

        return midError;
    }

    void collapseEdge(int v1, int v2, const glm::vec3 &targetPos)
    {
        vertexRemoved[v2] = true;

        // Update position of v1
        vertices[v1].Position = targetPos;

        // Update normal by averaging
        vertices[v1].Normal = glm::normalize(vertices[v1].Normal + vertices[v2].Normal);

        // Update quadric for v1
        quadrics[v1] = quadrics[v1] + quadrics[v2];

        // Update connections
        for (int v : vertexConnections[v2])
        {
            if (v != v1 && !vertexRemoved[v])
            {
                // Replace v2 with v1 in v's connections
                auto it = std::find(vertexConnections[v].begin(), vertexConnections[v].end(), v2);
                if (it != vertexConnections[v].end())
                {
                    *it = v1;
                }

                // Add v to v1's connections if not already there
                if (std::find(vertexConnections[v1].begin(), vertexConnections[v1].end(), v) 
                    == vertexConnections[v1].end())
                {
                    vertexConnections[v1].push_back(v);
                }
            }
        }

        // Sort and remove duplicates
        std::sort(vertexConnections[v1].begin(), vertexConnections[v1].end());
        vertexConnections[v1].erase(
            std::unique(vertexConnections[v1].begin(), vertexConnections[v1].end()),
            vertexConnections[v1].end());

        // Remove v1 from its own connections
        vertexConnections[v1].erase(
            std::remove(vertexConnections[v1].begin(), vertexConnections[v1].end(), v1),
            vertexConnections[v1].end());

        // Update boundary edges set if needed
        updateBoundaryEdgesAfterCollapse(v1, v2);
    }

    Mesh buildSimplifiedMesh()
    {
        std::vector<Vertex> newVertices;
        std::vector<unsigned int> newIndices;
        std::vector<int> vertexMap(vertices.size(), -1);

        // Add non-removed vertices
        for (size_t i = 0; i < vertices.size(); i++)
        {
            if (!vertexRemoved[i])
            {
                vertexMap[i] = newVertices.size();
                newVertices.push_back(vertices[i]);
            }
        }

        // Add valid triangles
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            int v1 = indices[i];
            int v2 = indices[i + 1];
            int v3 = indices[i + 2];

            // Check if this is a valid triangle
            if (!vertexRemoved[v1] && !vertexRemoved[v2] && !vertexRemoved[v3])
            {
                newIndices.push_back(vertexMap[v1]);
                newIndices.push_back(vertexMap[v2]);
                newIndices.push_back(vertexMap[v3]);
            }
            else
            {
                // Try to repair broken triangles by finding replacements
                int rep1 = vertexRemoved[v1] ? findReplacement(v1) : v1;
                int rep2 = vertexRemoved[v2] ? findReplacement(v2) : v2;
                int rep3 = vertexRemoved[v3] ? findReplacement(v3) : v3;

                // Only if all vertices have replacements and they form a non-degenerate triangle
                if (rep1 != -1 && rep2 != -1 && rep3 != -1 &&
                    rep1 != rep2 && rep2 != rep3 && rep3 != rep1)
                {
                    newIndices.push_back(vertexMap[rep1]);
                    newIndices.push_back(vertexMap[rep2]);
                    newIndices.push_back(vertexMap[rep3]);
                }
            }
        }

        return Mesh(newVertices, newIndices);
    }

    int findReplacement(int vertexIndex)
    {
        if (!vertexRemoved[vertexIndex])
            return vertexIndex;

        // Find a non-removed neighbor
        for (int neighbor : vertexConnections[vertexIndex])
        {
            if (!vertexRemoved[neighbor])
            {
                return neighbor;
            }
        }

        return -1; // No replacement found
    }

    bool isValidCollapse(int v1, int v2)
    {
        // Check if these vertices are connected
        auto it = std::find(vertexConnections[v1].begin(), vertexConnections[v1].end(), v2);
        if (it == vertexConnections[v1].end())
            return false;

        // Find common faces between v1 and v2
        std::vector<int> commonFaces;
        for (int f1 : vertexFaces[v1])
        {
            for (int f2 : vertexFaces[v2])
            {
                if (f1 == f2)
                {
                    commonFaces.push_back(f1);
                    break;
                }
            }
        }

        // Vertices should share at least one face
        if (commonFaces.empty())
            return false;

        // Don't collapse if either vertex is on a boundary
        if (isVertexOnBoundary(v1) || isVertexOnBoundary(v2))
            return false;

        // Get the target position for this collapse
        glm::vec3 newPos = (vertices[v1].Position + vertices[v2].Position) * 0.5f;

        // Check for normal flips
        for (int faceIdx : vertexFaces[v1])
        {
            if (std::find(commonFaces.begin(), commonFaces.end(), faceIdx) != commonFaces.end())
                continue; // Skip faces that will be removed by the collapse

            int i = faceIdx * 3;
            int f1 = indices[i];
            int f2 = indices[i + 1];
            int f3 = indices[i + 2];

            // Get the original normal
            glm::vec3 p1 = vertices[f1].Position;
            glm::vec3 p2 = vertices[f2].Position;
            glm::vec3 p3 = vertices[f3].Position;
            
            glm::vec3 origNormal = glm::normalize(glm::cross(p2 - p1, p3 - p1));

            // Get the new normal after collapse
            glm::vec3 newP1 = (f1 == v1) ? newPos : p1;
            glm::vec3 newP2 = (f2 == v1) ? newPos : p2;
            glm::vec3 newP3 = (f3 == v1) ? newPos : p3;
            
            glm::vec3 newNormal = glm::normalize(glm::cross(newP2 - newP1, newP3 - newP1));

            // If normals point in opposite directions, this is a flip
            if (glm::dot(origNormal, newNormal) < 0.5f) // Using threshold to be cautious
                return false;
        }

        // All checks passed
        return true;
    }

    std::set<std::pair<int, int>> identifyBoundaryEdges()
    {
        std::set<std::pair<int, int>> result;
        std::map<std::pair<int, int>, int> edgeCount;

        // Count how many times each edge appears
        int numFaces = indices.size() / 3;
        for (int f = 0; f < numFaces; f++)
        {
            int v1 = indices[f * 3];
            int v2 = indices[f * 3 + 1];
            int v3 = indices[f * 3 + 2];

            // Create edges with ordered vertices
            std::pair<int, int> e1 = std::minmax(v1, v2);
            std::pair<int, int> e2 = std::minmax(v2, v3);
            std::pair<int, int> e3 = std::minmax(v3, v1);

            edgeCount[e1]++;
            edgeCount[e2]++;
            edgeCount[e3]++;
        }

        // Edges appearing only once are boundary edges
        for (const auto& entry : edgeCount)
        {
            if (entry.second == 1)
            {
                result.insert(entry.first);
            }
        }

        return result;
    }

    void weldBoundaryVertices(float threshold)
    {
        // Extract all vertices on boundaries
        std::set<int> boundaryVerts;
        for (const auto& edge : boundaryEdges)
        {
            boundaryVerts.insert(edge.first);
            boundaryVerts.insert(edge.second);
        }

        // Convert to vector for easier indexing
        std::vector<int> boundaryVertList(boundaryVerts.begin(), boundaryVerts.end());
        
        // Find and weld close boundary vertices
        for (size_t i = 0; i < boundaryVertList.size(); i++)
        {
            for (size_t j = i+1; j < boundaryVertList.size(); j++)
            {
                int v1 = boundaryVertList[i];
                int v2 = boundaryVertList[j];
                
                if (vertexRemoved[v1] || vertexRemoved[v2])
                    continue;
                
                float dist = glm::distance(vertices[v1].Position, vertices[v2].Position);
                if (dist < threshold)
                {
                    // Weld v2 to v1
                    collapseEdge(v1, v2, vertices[v1].Position);
                }
            }
        }

        // Update boundary edges after welding
        boundaryEdges = identifyBoundaryEdges();
    }

    bool isBoundaryEdge(int v1, int v2)
    {
        std::pair<int, int> edge = std::minmax(v1, v2);
        return boundaryEdges.find(edge) != boundaryEdges.end();
    }

    bool isVertexOnBoundary(int v)
    {
        for (int neighbor : vertexConnections[v])
        {
            if (isBoundaryEdge(v, neighbor))
                return true;
        }
        return false;
    }

    void updateBoundaryEdgesAfterCollapse(int v1, int v2)
    {
        // Remove any boundary edges involving v2
        std::vector<std::pair<int, int>> toRemove;
        for (const auto& edge : boundaryEdges)
        {
            if (edge.first == v2 || edge.second == v2)
            {
                toRemove.push_back(edge);
            }
        }
        
        for (const auto& edge : toRemove)
        {
            boundaryEdges.erase(edge);
        }
        
        // If needed, we could recalculate boundary edges here:
        // boundaryEdges = identifyBoundaryEdges();
        // But for performance, we just update what we know
    }
};

#endif // QEM_H

#pragma once

#include "glew.h"
#include "glm.hpp"
#include <vector>

class TerrainGenerator {
public:
    TerrainGenerator(int width, int depth, float scale);
    ~TerrainGenerator();

    void generateTerrain();
    void render(GLuint program, const glm::mat4& viewProjectionMatrix, const glm::mat4& lightSpaceMatrix, GLuint depthMap);

private:
    int width, depth;
    float scale;
    std::vector<float> heights;
    GLuint terrainVAO, terrainVBO, terrainEBO;
    std::vector<unsigned int> indices;

    float generateHeight(int x, int z);
    void createTerrainMesh();
};

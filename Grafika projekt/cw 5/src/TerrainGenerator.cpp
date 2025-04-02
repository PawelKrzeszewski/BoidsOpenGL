#include "TerrainGenerator.hpp"
#include "glm.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>

float noise(int x, int z) {
    int n = x + z * 57;
    n = (n << 13) ^ n;
    return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}

float smoothNoise(int x, int z) {
    float corners = (noise(x - 1, z - 1) + noise(x + 1, z - 1) +
        noise(x - 1, z + 1) + noise(x + 1, z + 1)) / 16.0f;
    float sides = (noise(x - 1, z) + noise(x + 1, z) +
        noise(x, z - 1) + noise(x, z + 1)) / 8.0f;
    float center = noise(x, z) / 16.0f;
    return corners + sides + center;
}

float interpolate(float a, float b, float blend) {
    double theta = blend * M_PI;
    float f = (1.0f - cos(theta)) * 0.5f;
    return a * (1.0f - f) + b * f;
}

float interpolatedNoise(float x, float z) {
    int intX = (int)x;
    int intZ = (int)z;
    float fracX = x - intX;
    float fracZ = z - intZ;

    float v1 = smoothNoise(intX, intZ);
    float v2 = smoothNoise(intX + 1, intZ);
    float v3 = smoothNoise(intX, intZ + 1);
    float v4 = smoothNoise(intX + 1, intZ + 1);

    float i1 = interpolate(v1, v2, fracX);
    float i2 = interpolate(v3, v4, fracX);
    return interpolate(i1, i2, fracZ);
}

TerrainGenerator::TerrainGenerator(int width, int depth, float scale)
    : width(width), depth(depth), scale(scale) {
    generateTerrain();
}

TerrainGenerator::~TerrainGenerator() {
    glDeleteVertexArrays(1, &terrainVAO);
    glDeleteBuffers(1, &terrainVBO);
    glDeleteBuffers(1, &terrainEBO);
}

float TerrainGenerator::generateHeight(int x, int z) {
    return interpolatedNoise(x / 8.0f, z / 8.0f) * scale;
}

void TerrainGenerator::generateTerrain() {
    heights.resize(width * depth);
    for (int z = 0; z < depth; z++) {
        for (int x = 0; x < width; x++) {
            heights[z * width + x] = generateHeight(x, z);
        }
    }
    createTerrainMesh();
}

void TerrainGenerator::createTerrainMesh() {
    std::vector<float> data;  // [pos.x, pos.y, pos.z, normal.x, normal.y, normal.z]
    indices.clear();

    // Dla ka¿dego punktu w siatce
    for (int z = 0; z < depth; z++) {
        for (int x = 0; x < width; x++) {
            // Obliczenie pozycji
            float posX = (float)x - 25.0f;
            float posY = heights[z * width + x] - 2.0f;
            float posZ = (float)z - 25.0f;
            data.push_back(posX);
            data.push_back(posY);
            data.push_back(posZ);

            // Obliczenie normalnej – wykorzystujemy s¹siednie wysokoœci.
            // Dla brzegów wykorzystujemy wartoœæ bie¿¹cego punktu, aby unikn¹æ artefaktów.
            float heightL = (z > 0) ? heights[(z - 1) * width + x] : heights[z * width + x];
            float heightR = (z < depth - 1) ? heights[(z + 1) * width + x] : heights[z * width + x];
            float heightD = (x > 0) ? heights[z * width + (x - 1)] : heights[z * width + x];
            float heightU = (x < width - 1) ? heights[z * width + (x + 1)] : heights[z * width + x];

            glm::vec3 norm = glm::normalize(glm::vec3(heightD - heightU, 2.0f, heightL - heightR));
            data.push_back(norm.x);
            data.push_back(norm.y);
            data.push_back(norm.z);
        }
    }

    // Generowanie indeksów
    for (int z = 0; z < depth - 1; z++) {
        for (int x = 0; x < width - 1; x++) {
            int topLeft = z * width + x;
            int topRight = topLeft + 1;
            int bottomLeft = (z + 1) * width + x;
            int bottomRight = bottomLeft + 1;

            // Pierwszy trójk¹t
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            // Drugi trójk¹t
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    // Generowanie VAO, VBO i EBO
    glGenVertexArrays(1, &terrainVAO);
    glGenBuffers(1, &terrainVBO);
    glGenBuffers(1, &terrainEBO);

    glBindVertexArray(terrainVAO);

    glBindBuffer(GL_ARRAY_BUFFER, terrainVBO);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, terrainEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Ustawienie atrybutu 0: pozycja (pierwsze 3 floaty)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Ustawienie atrybutu 1: normalna (kolejne 3 floaty, offset = 3*sizeof(float))
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

// Funkcja render – przyjmuje macierz viewProjectionMatrix i ustawia uniformy.

void TerrainGenerator::render(GLuint program, const glm::mat4& viewProjectionMatrix, const glm::mat4& lightSpaceMatrix, GLuint depthMap) {
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "viewProjectionMatrix"), 1, GL_FALSE, &viewProjectionMatrix[0][0]);
    glUniform3f(glGetUniformLocation(program, "terrainColor"), 0.2f, 0.6f, 0.2f);
    glBindVertexArray(terrainVAO);
    glUniformMatrix4fv(glGetUniformLocation(program, "lightSpaceMatrix"), 1, GL_FALSE, &lightSpaceMatrix[0][0]);

    // Ustawienie tekstury depth mapy – przypisujemy j¹ do jednostki 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glUniform1i(glGetUniformLocation(program, "shadowMap"), 0);

    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

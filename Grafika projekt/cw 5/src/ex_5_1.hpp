#include "SOIL/SOIL.h"
#include "glew.h"
#include <GLFW/glfw3.h>
#include "glm.hpp"
#include "ext.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include "TerrainGenerator.hpp"
#include "Shader_Loader.h"
#include "Render_Utils.h"
#include "Camera.h"
#include "Texture.cpp"

#include "Box.cpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>

GLuint program;
GLuint program1;
GLuint programTex;
Core::Shader_Loader shaderLoader;
GLuint coneTextureID;
GLuint depthMapFBO;
GLuint depthMap;
GLuint shadowMapProgram;
const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
glm::vec3 lightPos = glm::vec3(0.0f, 10.0f, 0.0f);
Core::RenderContext shipContext;
Core::RenderContext sphereContext;
Core::RenderContext coneContext;
Core::RenderContext legofigureContext;
Core::RenderContext chairContext;

glm::vec3 cameraPos = glm::vec3(-4.f, 0, 0);
glm::vec3 cameraDir = glm::vec3(1.f, 0.f, 0.f);
glm::vec3 lightColor = glm::vec3(1.f, 1.f, 1.f);
glm::mat4 lightSpaceMatrix;
glm::vec3 spaceshipPos = glm::vec3(-4.f, 0, 0);
glm::vec3 spaceshipDir = glm::vec3(1.f, 0.f, 0.f);
GLuint VAO, VBO;
TerrainGenerator* terrain;
int WINDOW_WIDTH = 800;
int WINDOW_HEIGHT = 600;
bool normalMapping = true;

bool mousePressed = false;
double lastX = 0.0;
double lastY = 0.0;
float yaw = 0.0f;
float pitch = 0.0f;

bool cameraFollowsShip = true;
glm::vec3 cameraOffset = glm::vec3(0.0f, 1.0f, -0.3f);

float aspectRatio = 1.f;

float lastTime = -1.f;
float deltaTime = 0.f;

const float BOUNDARY_MIN = -3.0f;
const float BOUNDARY_MAX = 3.0f;

int NUM_BOIDS = 150;

namespace texture {
    GLuint cone;
    GLuint sphere;

    GLuint coneNormal;
    GLuint sphereNormal;
}

struct Boid {
    glm::vec3 position;
    glm::vec3 velocity;
};

std::vector<Boid> boids;

const float NEIGHBOR_RADIUS = 0.3f;
const float SEPARATION_RADIUS = 0.2f;

float MAX_SPEED = 3.0f;
float MIN_SPEED = 1.5f;
float CHANGE_SPEED = 0.0f;

float SEPARATION_WEIGHT = 1.2f;
float ALIGNMENT_WEIGHT = 1.2f;
float COHESION_WEIGHT = 1.9f;

const float COLLISION_RADIUS = 0.1f;
const float COLLISION_FORCE = 0.8f;

struct KDOP {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    float minDiag1, maxDiag1;
    float minDiag2, maxDiag2;
    float minDiag3, maxDiag3;

    KDOP() : minX(0), maxX(0), minY(0), maxY(0), minZ(0), maxZ(0),
        minDiag1(0), maxDiag1(0), minDiag2(0), maxDiag2(0), minDiag3(0), maxDiag3(0) {}

    void updateFromSphere(const glm::vec3& center, float radius) {

        minX = center.x - radius;
        maxX = center.x + radius;
        minY = center.y - radius;
        maxY = center.y + radius;
        minZ = center.z - radius;
        maxZ = center.z + radius;

        float diagRadius = radius * 1.414214f;
        minDiag1 = center.x + center.y - diagRadius;
        maxDiag1 = center.x + center.y + diagRadius;
        minDiag2 = center.y + center.z - diagRadius;
        maxDiag2 = center.y + center.z + diagRadius;
        minDiag3 = center.x + center.z - diagRadius;
        maxDiag3 = center.x + center.z + diagRadius;
    }
};

bool checkKDOPCollision(const KDOP& a, const KDOP& b) {
    return (a.minX <= b.maxX && a.maxX >= b.minX) &&
        (a.minY <= b.maxY && a.maxY >= b.minY) &&
        (a.minZ <= b.maxZ && a.maxZ >= b.minZ) &&
        (a.minDiag1 <= b.maxDiag1 && a.maxDiag1 >= b.minDiag1) &&
        (a.minDiag2 <= b.maxDiag2 && a.maxDiag2 >= b.minDiag2) &&
        (a.minDiag3 <= b.maxDiag3 && a.maxDiag3 >= b.minDiag3);
}

struct Obstacle {
    glm::vec3 position;
    float radius;
    KDOP boundingVolume;

    Obstacle(const glm::vec3& pos, float r) : position(pos), radius(r) {
        boundingVolume.updateFromSphere(position, radius);
    }
};

std::vector<Obstacle> obstacles;
void initShadowMapping() {
    // Configure depth map FBO
    glGenFramebuffers(1, &depthMapFBO);

    // Create depth texture
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    shadowMapProgram = shaderLoader.CreateProgram("shaders/shadow_map.vert", "shaders/shadow_map.frag");
}
void initObstacles() {

    obstacles.push_back(Obstacle(glm::vec3(0.0f, 0.0f, 0.0f), 0.3f));
    obstacles.push_back(Obstacle(glm::vec3(0.0f, 1.0f, 1.0f), 0.5f));
    obstacles.push_back(Obstacle(glm::vec3(2.0f, 0.0f, 0.0f), 1.3f));

}

void handleBoidObstacleCollisions(Boid& boid) {
    KDOP boidBounds;
    boidBounds.updateFromSphere(boid.position, COLLISION_RADIUS);

    for (const auto& obstacle : obstacles) {
        if (checkKDOPCollision(boidBounds, obstacle.boundingVolume)) {
            glm::vec3 toObstacle = obstacle.position - boid.position;
            float distance = glm::length(toObstacle);

            if (distance < (COLLISION_RADIUS + obstacle.radius)) {
                glm::vec3 collisionNormal = glm::normalize(toObstacle);

                glm::vec3 obstacleVelocity = glm::cross(glm::vec3(0, 1, 0), toObstacle) * 0.5f;
                glm::vec3 relativeVelocity = boid.velocity - obstacleVelocity;

                float approachSpeed = glm::dot(relativeVelocity, collisionNormal);

                if (approachSpeed > 0) {

                    glm::vec3 deflectionDir = glm::normalize(relativeVelocity - 2.0f * approachSpeed * collisionNormal);

                    float blendFactor = 0.3f;
                    boid.velocity = glm::normalize(
                        glm::mix(boid.velocity, deflectionDir * glm::length(boid.velocity), blendFactor)
                    ) * glm::length(boid.velocity);

                    float penetrationDepth = (COLLISION_RADIUS + obstacle.radius) - distance;
                    if (penetrationDepth > 0) {
                        boid.position -= collisionNormal * penetrationDepth;
                    }

                    float repulsionStrength = 2.0f;
                    boid.velocity += -collisionNormal * repulsionStrength;

                    float speed = glm::length(boid.velocity);
                    if (speed > MAX_SPEED) {
                        boid.velocity = glm::normalize(boid.velocity) * MAX_SPEED;
                    }
                }
            }
        }
    }
}


void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (mousePressed) {
        float sensitivity = 0.1f;
        float xoffset = (xpos - lastX) * sensitivity;
        float yoffset = (lastY - ypos) * sensitivity;
        lastX = xpos;
        lastY = ypos;

        yaw += xoffset;
        pitch += yoffset;

        pitch = glm::clamp(pitch, -89.0f, 89.0f);

        glm::vec3 direction;
        direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        direction.y = sin(glm::radians(pitch));
        direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraDir = glm::normalize(direction);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            mousePressed = true;
            glfwGetCursorPos(window, &lastX, &lastY);
        }
        else if (action == GLFW_RELEASE) {
            mousePressed = false;
        }
    }
}
void updateBoids() {
    for (size_t i = 0; i < boids.size(); ++i) {
        Boid& boid = boids[i];

        glm::vec3 separation(0.0f), alignment(0.0f), cohesion(0.0f);
        int separationCount = 0, alignmentCount = 0, cohesionCount = 0;

        handleBoidObstacleCollisions(boid);

        for (size_t j = 0; j < boids.size(); ++j) {
            if (i == j) continue;

            Boid& other = boids[j];
            float distance = glm::length(boid.position - other.position);

            if (distance < COLLISION_RADIUS) {
                glm::vec3 collisionNormal = glm::normalize(boid.position - other.position);
                boid.velocity += COLLISION_FORCE * collisionNormal;
                other.velocity -= COLLISION_FORCE * collisionNormal;

                float overlap = COLLISION_RADIUS - distance;
                boid.position += collisionNormal * (overlap / 2.0f);
                other.position -= collisionNormal * (overlap / 2.0f);
            }

            if (distance > 0 && distance < SEPARATION_RADIUS) {
                separation += (boid.position - other.position) / (distance * distance);
                separationCount++;
            }
            if (distance > 0 && distance < NEIGHBOR_RADIUS) {
                alignment += other.velocity;
                cohesion += other.position;
                alignmentCount++;
                cohesionCount++;
            }
        }

        if (separationCount > 0) separation = glm::normalize(separation / float(separationCount)) * MAX_SPEED;
        if (alignmentCount > 0) alignment = glm::normalize(alignment / float(alignmentCount)) * MAX_SPEED;
        if (cohesionCount > 0) cohesion = glm::normalize((cohesion / float(cohesionCount) - boid.position)) * MAX_SPEED;

        glm::vec3 newVelocity = boid.velocity +
            SEPARATION_WEIGHT * separation +
            ALIGNMENT_WEIGHT * alignment +
            COHESION_WEIGHT * cohesion +
            CHANGE_SPEED;

        float currentSpeed = glm::length(newVelocity);
        float targetSpeed = glm::clamp(currentSpeed, MIN_SPEED, MAX_SPEED);
        newVelocity = glm::normalize(newVelocity) * targetSpeed;

        boid.velocity = glm::mix(boid.velocity, newVelocity, 0.05f);

        boid.position += boid.velocity * deltaTime;

        if (boid.position.x < BOUNDARY_MIN || boid.position.x > BOUNDARY_MAX) {
            boid.velocity.x = -boid.velocity.x;
            boid.position.x = glm::clamp(boid.position.x, BOUNDARY_MIN, BOUNDARY_MAX);
        }
        if (boid.position.y < BOUNDARY_MIN || boid.position.y > BOUNDARY_MAX) {
            boid.velocity.y = -boid.velocity.y;
            boid.position.y = glm::clamp(boid.position.y, BOUNDARY_MIN, BOUNDARY_MAX);
        }
        if (boid.position.z < BOUNDARY_MIN || boid.position.z > BOUNDARY_MAX) {
            boid.velocity.z = -boid.velocity.z;
            boid.position.z = glm::clamp(boid.position.z, BOUNDARY_MIN, BOUNDARY_MAX);
        }
    }
}
void updateDeltaTime(float time) {
    if (lastTime < 0) {
        lastTime = time;
        return;
    }

    deltaTime = time - lastTime;
    if (deltaTime > 0.1) deltaTime = 0.1;
    lastTime = time;
}

glm::mat4 createCameraMatrix() {
    glm::vec3 cameraSide = glm::normalize(glm::cross(cameraDir, glm::vec3(0.f, 1.f, 0.f)));
    glm::vec3 cameraUp = glm::normalize(glm::cross(cameraSide, cameraDir));
    glm::mat4 cameraRotationMatrix = glm::mat4({
        cameraSide.x, cameraSide.y, cameraSide.z, 0,
        cameraUp.x, cameraUp.y, cameraUp.z, 0,
        -cameraDir.x, -cameraDir.y, -cameraDir.z, 0,
        0., 0., 0., 1.,
        });
    cameraRotationMatrix = glm::transpose(cameraRotationMatrix);
    glm::mat4 cameraMatrix = cameraRotationMatrix * glm::translate(-cameraPos);

    return cameraMatrix;
}

glm::mat4 createPerspectiveMatrix() {
    glm::mat4 perspectiveMatrix;
    float n = 0.05;
    float f = 200.;
    float a1 = glm::min(aspectRatio, 1.f);
    float a2 = glm::min(1 / aspectRatio, 1.f);
    perspectiveMatrix = glm::mat4({
        1, 0., 0., 0.,
        0., aspectRatio, 0., 0.,
        0., 0., (f + n) / (n - f), 2 * f * n / (n - f),
        0., 0., -1., 0.,
        });

    perspectiveMatrix = glm::transpose(perspectiveMatrix);

    return perspectiveMatrix;
}

void drawObjectColor(Core::RenderContext& context, glm::mat4 modelMatrix, glm::vec3 color, bool shadowPass = false) {
    if (shadowPass) {
        glUseProgram(shadowMapProgram);
        glm::mat4 lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 1.0f, 7.5f);
        glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
        glm::mat4 lightSpaceMatrix = lightProjection * lightView;

        glUniformMatrix4fv(glGetUniformLocation(shadowMapProgram, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
        glUniformMatrix4fv(glGetUniformLocation(shadowMapProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
    }
    else {
        glUseProgram(program);
        glm::mat4 viewProjectionMatrix = createPerspectiveMatrix() * createCameraMatrix();
        glm::mat4 transformation = viewProjectionMatrix * modelMatrix;

        glm::mat4 lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 1.0f, 7.5f);
        glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
        glm::mat4 lightSpaceMatrix = lightProjection * lightView;

        glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, glm::value_ptr(transformation));
        glUniformMatrix4fv(glGetUniformLocation(program, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
        glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
        glUniform3fv(glGetUniformLocation(program, "color"), 1, glm::value_ptr(color));
        glUniform3fv(glGetUniformLocation(program, "lightPos"), 1, glm::value_ptr(lightPos));
        glUniform3fv(glGetUniformLocation(program, "viewPos"), 1, glm::value_ptr(cameraPos));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glUniform1i(glGetUniformLocation(program, "shadowMap"), 0);
    }

    Core::DrawContext(context);
    glUseProgram(0);
}

void drawObjectTexture(Core::RenderContext& context, glm::mat4 modelMatrix, GLuint textureID, GLuint textureNormalID) {
    glUseProgram(programTex);
    glm::mat4 viewProjectionMatrix = createPerspectiveMatrix() * createCameraMatrix();
    glm::mat4 transformation = viewProjectionMatrix * modelMatrix;
    glUniformMatrix4fv(glGetUniformLocation(programTex, "transformation"), 1, GL_FALSE, (float*)&transformation);
    glUniformMatrix4fv(glGetUniformLocation(programTex, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);
    glUniform3f(glGetUniformLocation(programTex, "lightPos"), lightPos[0], lightPos[1], lightPos[2]);

    Core::SetActiveTexture(textureID, "colorTexture", programTex, 0);
    Core::SetActiveTexture(textureNormalID, "normalMap", programTex, 1);
    Core::DrawContext(context);

}

void renderBoids() {
    for (auto& boid : boids) {

        glm::vec3 forward = glm::normalize(boid.velocity);

        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(up, forward));

        up = glm::cross(forward, right);

        glm::mat4 rotationMatrix = glm::mat4(
            glm::vec4(right, 0.0f),
            glm::vec4(up, 0.0f),
            glm::vec4(forward, 0.0f),
            glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );

        glm::mat4 baseRotation = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

        glm::mat4 modelMatrix = glm::translate(boid.position) * rotationMatrix * baseRotation * glm::scale(glm::vec3(0.06f));

        //texture!
        drawObjectTexture(coneContext, modelMatrix, texture::cone, texture::coneNormal);
    }
}

void drawBoundaryCube() {
    glUseProgram(program);

    std::vector<glm::vec3> vertices = {
        {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MIN}, {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MIN},
        {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MAX}, {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MAX},
        {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MIN}, {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MIN},
        {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MAX}, {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MAX},

        {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MIN}, {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MIN},
        {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MIN}, {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MIN},
        {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MAX}, {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MAX},
        {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MAX}, {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MAX},

        {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MIN}, {BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MAX},
        {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MIN}, {BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MAX},
        {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MIN}, {BOUNDARY_MIN, BOUNDARY_MAX, BOUNDARY_MAX},
        {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MIN}, {BOUNDARY_MAX, BOUNDARY_MAX, BOUNDARY_MAX}
    };

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    glm::mat4 viewProjectionMatrix = createPerspectiveMatrix() * createCameraMatrix();
    glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, (float*)&viewProjectionMatrix);

    glUniform3f(glGetUniformLocation(program, "color"), 1.0f, 1.0f, 1.0f);

    glDrawArrays(GL_LINES, 0, vertices.size());

    glBindVertexArray(0);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);

    glUseProgram(0);
}

void renderScene(GLFWwindow* window) {
    float time = glfwGetTime();
    updateDeltaTime(time);

    // 1. First render to depth map
    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    for (size_t i = 0; i < obstacles.size(); i++) {
        float orbitRadius = 1.5f + i * 0.5f;
        float rotationSpeed = 1.0f - i * 0.2f;
        float angle = time * rotationSpeed;

        glm::mat4 orbitTranslation = glm::translate(glm::mat4(1.0f),
            glm::vec3(
                orbitRadius * cos(angle),
                0.5f * sin(time * 0.5f),
                orbitRadius * sin(angle)
            ));

        glm::mat4 scaleMatrix = glm::scale(glm::vec3(obstacles[i].radius));
        glm::mat4 modelMatrix = orbitTranslation * scaleMatrix;

        drawObjectTexture(coneContext, modelMatrix, texture::sphere, texture::sphereNormal);
    }

    // Render boids for shadow mapping
    for (auto& boid : boids) {
        glm::vec3 forward = glm::normalize(boid.velocity);
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(up, forward));
        up = glm::cross(forward, right);

        glm::mat4 rotationMatrix = glm::mat4(
            glm::vec4(right, 0.0f),
            glm::vec4(up, 0.0f),
            glm::vec4(forward, 0.0f),
            glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );

        glm::mat4 baseRotation = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 modelMatrix = glm::translate(boid.position) * rotationMatrix * baseRotation * glm::scale(glm::vec3(0.06f));
        //

        drawObjectTexture(coneContext, modelMatrix, texture::cone, texture::coneNormal);

    }

    // 2. Render scene as normal with shadow mapping
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, 1920, 1080);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.3f, 0.3f, 1.0f);

    // Render objects with shadows
    for (size_t i = 0; i < obstacles.size(); i++) {
        float orbitRadius = 1.5f + i * 0.5f;
        float rotationSpeed = 1.0f - i * 0.2f;
        float angle = time * rotationSpeed;

        glm::mat4 orbitTranslation = glm::translate(glm::mat4(1.0f),
            glm::vec3(
                orbitRadius * cos(angle),
                0.5f * sin(time * 0.5f),
                orbitRadius * sin(angle)
            ));

        glm::mat4 scaleMatrix = glm::scale(glm::vec3(obstacles[i].radius));
        glm::mat4 modelMatrix = orbitTranslation * scaleMatrix;

        obstacles[i].position = glm::vec3(
            orbitRadius * cos(angle),
            0.5f * sin(time * 0.5f),
            orbitRadius * sin(angle)
        );

        obstacles[i].boundingVolume.updateFromSphere(obstacles[i].position, obstacles[i].radius);

        drawObjectTexture(sphereContext, modelMatrix, texture::sphere, texture::sphereNormal);
    }
    
    // Obliczenie macierzy przestrzeni światła (przyjęte parametry mogą być inne – dostosuj je do sceny)
    glm::mat4 lightProjection = glm::ortho(-50.0f, 50.0f, -50.0f, 50.0f, -30.0f, 50.0f);
    glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    lightSpaceMatrix = lightProjection * lightView;

    

    drawBoundaryCube();
    glm::mat4 viewProjectionMatrix = createPerspectiveMatrix() * createCameraMatrix();
    terrain->render(program1, viewProjectionMatrix, lightSpaceMatrix, depthMapFBO);
    updateBoids();
    renderBoids();

    glfwSwapBuffers(window);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    WINDOW_WIDTH = width;
    WINDOW_HEIGHT = height;
    aspectRatio = width / float(height);
    glViewport(0, 0, width, height);
}

void loadModelToContext(std::string path, Core::RenderContext& context) {
    Assimp::Importer import;
    const aiScene * scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_CalcTangentSpace);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
        return;
    }
    context.initFromAssimpMesh(scene->mMeshes[0]);
}

void init(GLFWwindow* window) {
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glEnable(GL_DEPTH_TEST);
    program = shaderLoader.CreateProgram("shaders/shader_5_1.vert", "shaders/shader_5_1.frag");
    program1 = shaderLoader.CreateProgram("shaders/terrain.vert", "shaders/terrain.frag");
    programTex = shaderLoader.CreateProgram("shaders/shader_tex.vert", "shaders/shader_tex.frag");
    terrain = new TerrainGenerator(256, 256, 5.0f);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    loadModelToContext("./models/sphere.obj", sphereContext);
    loadModelToContext("./models/spaceship.obj", shipContext);
    loadModelToContext("./models/cone.obj", coneContext);
    loadModelToContext("./models/legofigure.obj", legofigureContext);
    //loadModelToContext("./models/chair.obj", chairContext);

    texture::cone = Core::LoadTexture("img/CONE.jpg");
    texture::coneNormal = Core::LoadTexture("img/CONE_NORMAL.jpg");
    texture::sphere = Core::LoadTexture("img/SPHERE.jpg");
    texture::sphereNormal = Core::LoadTexture("img/SPHERE_NORMAL.jpg");

    initObstacles();
    for (int i = 0; i < NUM_BOIDS; i++) {
        Boid boid;

        boid.position = glm::vec3(
            BOUNDARY_MIN + static_cast<float>(rand()) / RAND_MAX * (BOUNDARY_MAX - BOUNDARY_MIN),
            BOUNDARY_MIN + static_cast<float>(rand()) / RAND_MAX * (BOUNDARY_MAX - BOUNDARY_MIN),
            BOUNDARY_MIN + static_cast<float>(rand()) / RAND_MAX * (BOUNDARY_MAX - BOUNDARY_MIN)
        );

        glm::vec3 randomDirection = glm::normalize(glm::vec3(
            (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f,
            (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f,
            (static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f
        ));

        float randomSpeed = MIN_SPEED + static_cast<float>(rand()) / RAND_MAX * (MAX_SPEED - MIN_SPEED);
        boid.velocity = randomDirection * randomSpeed;

        boids.push_back(boid);
    }
}

void shutdown(GLFWwindow* window) {
    shaderLoader.DeleteProgram(program);
    delete terrain;
}
static bool wasPressed = false;

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    int valuesChangeMultiplier = 1;
    float change = 0.05;

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        valuesChangeMultiplier = 10;
    }

    if (key == GLFW_KEY_N && action == GLFW_PRESS)
    {
        if (normalMapping)
        {
            normalMapping = false;
            texture::coneNormal = Core::LoadTexture("img/BLUE.jpg");
            texture::sphereNormal = Core::LoadTexture("img/BLUE.jpg");
            std::cout << "Normal mapping turned OFF" << std::endl;
        }
        else
        {
            normalMapping = true;
            texture::coneNormal = Core::LoadTexture("img/CONE_NORMAL.jpg");
            texture::sphereNormal = Core::LoadTexture("img/SPHERE_NORMAL.jpg");
            std::cout << "Normal mapping turned ON" << std::endl;
        }
        
    }

    if (key == GLFW_KEY_T && action == GLFW_PRESS)
    {
        SEPARATION_WEIGHT += (change * valuesChangeMultiplier);
        std::cout << "Current SEPARATION_WEIGHT: " << SEPARATION_WEIGHT << std::endl;
    }
    if (key == GLFW_KEY_G && action == GLFW_PRESS)
    {
        if (valuesChangeMultiplier > 1) {
            if (SEPARATION_WEIGHT >= (change * valuesChangeMultiplier) + change) {
                SEPARATION_WEIGHT -= (change * valuesChangeMultiplier);
            }
            else {
                std::cout << "SEPARATION_WEIGHT at its minimum value!" << std::endl;
            }
        }
        else {
            if (SEPARATION_WEIGHT >= 2 * change) {
                SEPARATION_WEIGHT -= change;
            }
            else {
                std::cout << "SEPARATION_WEIGHT at its minimum value!" << std::endl;
            }
        }
        std::cout << "Current SEPARATION_WEIGHT: " << SEPARATION_WEIGHT << std::endl;
    }

    if (key == GLFW_KEY_Y && action == GLFW_PRESS)
    {
        ALIGNMENT_WEIGHT += (change * valuesChangeMultiplier);
        std::cout << "Current ALIGNMENT_WEIGHT: " << ALIGNMENT_WEIGHT << std::endl;
    }

    if (key == GLFW_KEY_H && action == GLFW_PRESS)
    {
        if (valuesChangeMultiplier > 1) {
            if (ALIGNMENT_WEIGHT >= (change * valuesChangeMultiplier) + change) {
                ALIGNMENT_WEIGHT -= (change * valuesChangeMultiplier);
            }
            else {
                std::cout << "ALIGNMENT_WEIGHT at its minimum value!" << std::endl;
            }
        }
        else {
            if (ALIGNMENT_WEIGHT >= 2 * change) {
                ALIGNMENT_WEIGHT -= change;
            }
            else {
                std::cout << "ALIGNMENT_WEIGHT at its minimum value!" << std::endl;
            }
        }
        std::cout << "Current ALIGNMENT_WEIGHT: " << ALIGNMENT_WEIGHT << std::endl;
    }

    if (key == GLFW_KEY_U && action == GLFW_PRESS)
    {
        COHESION_WEIGHT += change * valuesChangeMultiplier;
        std::cout << "Current COHESION_WEIGHT: " << COHESION_WEIGHT << std::endl;
    }

    if (key == GLFW_KEY_J && action == GLFW_PRESS)
    {
        if (valuesChangeMultiplier > 1) {
            if (COHESION_WEIGHT >= (change * valuesChangeMultiplier) + change) {
                COHESION_WEIGHT -= (change * valuesChangeMultiplier);
            }
            else {
                std::cout << "COHESION_WEIGHT at its minimum value!" << std::endl;
            }
        }
        else {
            if (COHESION_WEIGHT >= 2 * change) {
                COHESION_WEIGHT -= change;
            }
            else {
                std::cout << "COHESION_WEIGHT at its minimum value!" << std::endl;
            }
        }
        std::cout << "Current COHESION_WEIGHT: " << COHESION_WEIGHT << std::endl;
    }

    if (key == GLFW_KEY_I && action == GLFW_PRESS)
    {
        CHANGE_SPEED += 0.05 * valuesChangeMultiplier;
        MIN_SPEED += 0.05 * valuesChangeMultiplier;
        MAX_SPEED += 0.05 * valuesChangeMultiplier;
        std::cout << "Current CHANGE_SPEED: " << CHANGE_SPEED << "Current MIN_SPEED and MAX_SPEED: " << MIN_SPEED << ", " << MAX_SPEED << std::endl;
    }

    if (key == GLFW_KEY_K && action == GLFW_PRESS)
    {
        if (valuesChangeMultiplier > 1) {
            if (MIN_SPEED >= (change * valuesChangeMultiplier) + change) {
                CHANGE_SPEED -= (change * valuesChangeMultiplier);
                MIN_SPEED -= (change * valuesChangeMultiplier);
                MAX_SPEED -= (change * valuesChangeMultiplier);
            }
            else {
                std::cout << "MIN_SPEED at its minimum value!" << std::endl;
            }
        }
        else {
            if (MIN_SPEED >= 2 * change) {
                CHANGE_SPEED -= change;
                MIN_SPEED -= change;
                MAX_SPEED -= change;
            }
            else {
                std::cout << "MIN_SPEED at its minimum value!" << std::endl;
            }
        }
        
        std::cout << "Current CHANGE_SPEED: " << CHANGE_SPEED << "Current MIN_SPEED and MAX_SPEED: " << MIN_SPEED << ", " << MAX_SPEED << std::endl;
    }

}

void processInput(GLFWwindow* window) {
    float moveSpeed = 0.05f * deltaTime * 60;

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        moveSpeed *= 3;
    }

    

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    glm::vec3 cameraSide = glm::normalize(glm::cross(cameraDir, glm::vec3(0.f, 1.f, 0.f)));
    glm::vec3 cameraUp = glm::vec3(0.f, 1.f, 0.f);

    //Obsługa wejścia - interaktywność

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraDir * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraDir * moveSpeed;

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += cameraSide * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= cameraSide * moveSpeed;

    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        cameraPos += cameraUp * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        cameraPos -= cameraUp * moveSpeed;

    //Zmiana parametrów do algorytmu Boids
    glfwSetKeyCallback(window, keyCallback);

    spaceshipPos = cameraPos;
    spaceshipDir = cameraDir;
}

void renderLoop(GLFWwindow* window) {
    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        renderScene(window);
        glfwPollEvents();
    }
}